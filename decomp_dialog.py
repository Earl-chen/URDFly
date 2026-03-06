#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import xml.etree.ElementTree as ET

from PyQt5.QtWidgets import (
    QDialog,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QLabel,
    QSpinBox,
    QCheckBox,
    QProgressBar,
    QApplication,
    QTextBrowser,
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor

import trimesh

from simplify_mesh import is_vhacd_available
from decomp_worker import MeshInfo, DecompWorker
from translations import tr
from widgets import CollapsibleSection


# ---------------------------------------------------------------------------
# Dialog states
# ---------------------------------------------------------------------------
STATE_CONFIGURE = 0
STATE_PROCESSING = 1
STATE_RESULTS = 2

# Table column indices (configure / results share the first columns)
COL_CHECK = 0
COL_LINK = 1
COL_FILE = 2
COL_ORIG_FACES = 3
COL_MAX_HULLS = 4
# Results-only columns (appended dynamically)
COL_RES_HULLS = 5
COL_RES_FACES = 6
COL_RES_STATUS = 7


class DecompDialog(QDialog):
    """Three-state dialog for convex decomposition of collision meshes.

    States
    ------
    CONFIGURE  – user picks meshes, sets per-mesh maxConvexHulls
    PROCESSING – worker thread runs, progress bar updates
    RESULTS    – shows result hull/face counts and status
    """

    decomposition_applied = pyqtSignal(str)  # emits modified URDF path

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------
    def __init__(self, parent, collision_mesh_files, link_names, urdf_file):
        super().__init__(parent)
        self.setWindowFlags(
            Qt.Window
            | Qt.WindowCloseButtonHint
            | Qt.WindowMinMaxButtonsHint
        )

        self._collision_mesh_files = collision_mesh_files or []
        self._link_names = link_names or []
        self._urdf_file = urdf_file
        self._mesh_infos = []        # [MeshInfo, ...]
        self._worker = None
        self._state = STATE_CONFIGURE

        self._init_ui()
        self._populate_table()

    # ------------------------------------------------------------------
    # UI setup
    # ------------------------------------------------------------------
    def _init_ui(self):
        self.setWindowTitle(tr("decomp_title"))
        self.setMinimumWidth(820)
        self.setMinimumHeight(480)

        layout = QVBoxLayout(self)

        # V-HACD warning
        self._vhacd_available = is_vhacd_available()
        self._warn_label = QLabel()
        self._warn_label.setWordWrap(True)
        self._warn_label.setStyleSheet(
            "QLabel { color: #FFB74D; padding: 6px; "
            "border: 1px solid #FFB74D; border-radius: 4px; }"
        )
        layout.addWidget(self._warn_label)
        if self._vhacd_available:
            self._warn_label.hide()
        else:
            self._warn_label.setText(tr("vhacd_missing_warning"))

        # --- Collapsible help panel ---
        self._help_section = CollapsibleSection(
            tr("decomp_help_title"), parent=self, expanded=False
        )
        help_browser = QTextBrowser()
        help_browser.setOpenExternalLinks(True)
        help_browser.setMaximumHeight(260)
        help_browser.setHtml(tr("decomp_help_html"))
        self._help_section.add_widget(help_browser)
        layout.addWidget(self._help_section)

        # --- Global settings row ---
        global_row = QHBoxLayout()
        global_row.addWidget(QLabel(tr("decomp_default_hulls")))
        self._global_hulls_spin = QSpinBox()
        self._global_hulls_spin.setRange(1, 256)
        self._global_hulls_spin.setValue(32)
        self._global_hulls_spin.valueChanged.connect(self._on_global_hulls_changed)
        global_row.addWidget(self._global_hulls_spin)

        global_row.addStretch()

        self._btn_select_all = QPushButton(tr("decomp_select_all"))
        self._btn_select_all.clicked.connect(self._on_select_all)
        global_row.addWidget(self._btn_select_all)

        self._btn_deselect_all = QPushButton(tr("decomp_deselect_all"))
        self._btn_deselect_all.clicked.connect(self._on_deselect_all)
        global_row.addWidget(self._btn_deselect_all)

        layout.addLayout(global_row)

        # --- Table ---
        self._table = QTableWidget()
        self._table.setSelectionBehavior(QTableWidget.SelectRows)
        layout.addWidget(self._table)

        # --- Progress bar + label ---
        self._progress_bar = QProgressBar()
        self._progress_bar.hide()
        layout.addWidget(self._progress_bar)

        self._progress_label = QLabel()
        self._progress_label.hide()
        layout.addWidget(self._progress_label)

        # --- Buttons ---
        btn_row = QHBoxLayout()
        btn_row.addStretch()

        self._btn_cancel = QPushButton(tr("btn_cancel"))
        self._btn_cancel.clicked.connect(self._on_cancel_clicked)
        btn_row.addWidget(self._btn_cancel)

        self._btn_action = QPushButton(tr("decomp_start"))
        self._btn_action.clicked.connect(self._on_action_clicked)
        btn_row.addWidget(self._btn_action)

        layout.addLayout(btn_row)

    # ------------------------------------------------------------------
    # Table population
    # ------------------------------------------------------------------
    def _populate_table(self):
        """Build table rows from collision_mesh_files + link_names."""
        rows = len(self._collision_mesh_files)
        self._table.setColumnCount(5)
        self._table.setHorizontalHeaderLabels([
            "",                         # checkbox
            tr("decomp_link_name"),
            tr("mesh_file"),
            tr("decomp_original_faces"),
            tr("max_convex_hulls"),
        ])
        self._table.setRowCount(rows)

        self._mesh_infos.clear()

        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            for i, mesh_file in enumerate(self._collision_mesh_files):
                link = self._link_names[i] if i < len(self._link_names) else ""
                info = MeshInfo(mesh_file, link)
                self._mesh_infos.append(info)

                # Checkbox
                chk = QCheckBox()
                chk.setChecked(True)
                chk.stateChanged.connect(
                    lambda state, idx=i: self._on_check_changed(idx, state)
                )
                self._table.setCellWidget(i, COL_CHECK, chk)

                # Link name
                item = QTableWidgetItem(link)
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self._table.setItem(i, COL_LINK, item)

                # File name
                item = QTableWidgetItem(info.file_name)
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                item.setToolTip(mesh_file)
                self._table.setItem(i, COL_FILE, item)

                # Original face count (loaded here)
                face_count = 0
                try:
                    mesh = trimesh.load(mesh_file)
                    face_count = int(mesh.faces.shape[0])
                except Exception:
                    pass
                info.original_faces = face_count
                item = QTableWidgetItem(str(face_count))
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self._table.setItem(i, COL_ORIG_FACES, item)

                # Max convex hulls spin
                spin = QSpinBox()
                spin.setRange(1, 256)
                spin.setValue(32)
                spin.valueChanged.connect(
                    lambda val, idx=i: self._on_hull_spin_changed(idx, val)
                )
                self._table.setCellWidget(i, COL_MAX_HULLS, spin)
        finally:
            QApplication.restoreOverrideCursor()

        # Resize columns
        self._table.horizontalHeader().setSectionResizeMode(COL_CHECK, QHeaderView.ResizeToContents)
        self._table.horizontalHeader().setSectionResizeMode(COL_LINK, QHeaderView.Stretch)
        self._table.horizontalHeader().setSectionResizeMode(COL_FILE, QHeaderView.Stretch)
        self._table.horizontalHeader().setSectionResizeMode(COL_ORIG_FACES, QHeaderView.ResizeToContents)
        self._table.horizontalHeader().setSectionResizeMode(COL_MAX_HULLS, QHeaderView.ResizeToContents)

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------
    def _set_state(self, state):
        self._state = state

        if state == STATE_CONFIGURE:
            self._progress_bar.hide()
            self._progress_label.hide()
            self._btn_action.setText(tr("decomp_start"))
            self._btn_action.setEnabled(True)
            self._btn_select_all.setEnabled(True)
            self._btn_deselect_all.setEnabled(True)
            self._global_hulls_spin.setEnabled(True)
            self._set_table_editable(True)
            # Remove result columns if they exist
            while self._table.columnCount() > 5:
                self._table.removeColumn(self._table.columnCount() - 1)

        elif state == STATE_PROCESSING:
            self._progress_bar.setValue(0)
            self._progress_bar.show()
            self._progress_label.show()
            self._btn_action.setEnabled(False)
            self._btn_select_all.setEnabled(False)
            self._btn_deselect_all.setEnabled(False)
            self._global_hulls_spin.setEnabled(False)
            self._set_table_editable(False)

        elif state == STATE_RESULTS:
            self._progress_bar.hide()
            self._progress_label.hide()
            self._btn_action.setText(tr("decomp_apply"))
            self._btn_action.setEnabled(True)
            self._warn_label.hide()
            self._show_results()

    def _set_table_editable(self, editable):
        """Enable/disable all interactive widgets in the table."""
        for row in range(self._table.rowCount()):
            chk = self._table.cellWidget(row, COL_CHECK)
            if chk:
                chk.setEnabled(editable)
            spin = self._table.cellWidget(row, COL_MAX_HULLS)
            if spin:
                spin.setEnabled(editable)

    # ------------------------------------------------------------------
    # Slot: global hulls spin
    # ------------------------------------------------------------------
    def _on_global_hulls_changed(self, value):
        for row in range(self._table.rowCount()):
            spin = self._table.cellWidget(row, COL_MAX_HULLS)
            if spin:
                spin.setValue(value)
            if row < len(self._mesh_infos):
                self._mesh_infos[row].max_convex_hulls = value

    # ------------------------------------------------------------------
    # Slot: per-row checkbox / spinbox
    # ------------------------------------------------------------------
    def _on_check_changed(self, idx, state):
        if idx < len(self._mesh_infos):
            self._mesh_infos[idx].enabled = (state == Qt.Checked)

    def _on_hull_spin_changed(self, idx, value):
        if idx < len(self._mesh_infos):
            self._mesh_infos[idx].max_convex_hulls = value

    def _on_select_all(self):
        for row in range(self._table.rowCount()):
            chk = self._table.cellWidget(row, COL_CHECK)
            if chk:
                chk.setChecked(True)

    def _on_deselect_all(self):
        for row in range(self._table.rowCount()):
            chk = self._table.cellWidget(row, COL_CHECK)
            if chk:
                chk.setChecked(False)

    # ------------------------------------------------------------------
    # Decomposition lifecycle
    # ------------------------------------------------------------------
    def _start_decomposition(self):
        self._set_state(STATE_PROCESSING)
        self._worker = DecompWorker(self._mesh_infos, parent=self)
        self._worker.mesh_started.connect(self._on_mesh_started)
        self._worker.mesh_finished.connect(self._on_mesh_finished)
        self._worker.overall_progress.connect(self._on_overall_progress)
        self._worker.all_finished.connect(self._on_all_finished)
        self._worker.start()

    def _cancel_decomposition(self):
        if self._worker:
            self._worker.cancel()

    # ------------------------------------------------------------------
    # Worker signal handlers
    # ------------------------------------------------------------------
    def _on_mesh_started(self, idx):
        self._table.selectRow(idx)
        enabled_count = sum(1 for m in self._mesh_infos if m.enabled)
        done = sum(1 for m in self._mesh_infos[:idx] if m.enabled and m.status != "pending")
        info = self._mesh_infos[idx]
        self._progress_label.setText(
            tr("decomp_processing", None, done + 1, enabled_count, info.file_name)
        )

    def _on_mesh_finished(self, idx, info):
        pass  # results are written in _show_results

    def _on_overall_progress(self, current, total):
        if total > 0:
            self._progress_bar.setValue(int(current * 100 / total))

    def _on_all_finished(self, mesh_infos):
        self._worker = None
        self._set_state(STATE_RESULTS)

    # ------------------------------------------------------------------
    # Results display
    # ------------------------------------------------------------------
    def _show_results(self):
        """Add result columns and fill them; show fallback warning if needed."""
        # Add result columns
        self._table.setColumnCount(8)
        self._table.setHorizontalHeaderItem(
            COL_RES_HULLS, QTableWidgetItem(tr("decomp_result_hulls"))
        )
        self._table.setHorizontalHeaderItem(
            COL_RES_FACES, QTableWidgetItem(tr("decomp_result_faces"))
        )
        self._table.setHorizontalHeaderItem(
            COL_RES_STATUS, QTableWidgetItem(tr("decomp_status"))
        )

        fallback_count = 0

        for row, info in enumerate(self._mesh_infos):
            # Result hull count
            if info.status in ("success", "fallback"):
                txt = str(info.result_hull_count)
            else:
                txt = "--"
            item = QTableWidgetItem(txt)
            item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self._table.setItem(row, COL_RES_HULLS, item)

            # Result face count + reduction tooltip
            if info.status in ("success", "fallback"):
                txt = str(info.result_faces)
                if info.original_faces > 0:
                    pct = round(
                        (1 - info.result_faces / info.original_faces) * 100, 1
                    )
                    tooltip = tr("decomp_reduction_tooltip", None, info.result_faces, pct)
                else:
                    tooltip = ""
            else:
                txt = "--"
                tooltip = ""
            item = QTableWidgetItem(txt)
            item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            if tooltip:
                item.setToolTip(tooltip)
            self._table.setItem(row, COL_RES_FACES, item)

            # Status label
            status_map = {
                "success": (tr("decomp_status_success"), QColor("#4CAF50")),
                "fallback": (tr("decomp_status_fallback"), QColor("#FFB74D")),
                "error": (tr("decomp_status_error"), QColor("#EF5350")),
                "skipped": (tr("decomp_status_skipped"), QColor("#9E9E9E")),
                "pending": (tr("decomp_status_pending"), QColor("#9E9E9E")),
            }
            label, color = status_map.get(
                info.status, (info.status, QColor("#9E9E9E"))
            )
            item = QTableWidgetItem(label)
            item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            item.setForeground(color)
            if info.error_message:
                item.setToolTip(info.error_message)
            self._table.setItem(row, COL_RES_STATUS, item)

            if info.status == "fallback":
                fallback_count += 1

        # Resize new columns
        for col in (COL_RES_HULLS, COL_RES_FACES, COL_RES_STATUS):
            self._table.horizontalHeader().setSectionResizeMode(
                col, QHeaderView.ResizeToContents
            )

        # Fallback warning banner
        if fallback_count > 0:
            self._warn_label.setText(
                tr("decomp_fallback_warning", None, fallback_count)
            )
            self._warn_label.show()

    # ------------------------------------------------------------------
    # Apply to URDF
    # ------------------------------------------------------------------
    def _apply_to_urdf(self):
        """Replace collision mesh references in the URDF with _approx versions.

        Only meshes with status ``success`` or ``fallback`` are replaced.
        The modified URDF is saved as ``<name>_temp.urdf`` (non-destructive).
        """
        # Build a mapping: original_filename_lower → output_file
        replaced = {}
        for info in self._mesh_infos:
            if info.status in ("success", "fallback") and info.output_file:
                # key: basename of the original mesh, lowered for robust matching
                replaced[os.path.basename(info.file_path).lower()] = info.output_file

        if not replaced:
            return

        tree = ET.parse(self._urdf_file)
        root = tree.getroot()

        for collision in root.iter("collision"):
            mesh_elem = collision.find(".//mesh")
            if mesh_elem is None:
                continue
            filename_attr = mesh_elem.get("filename")
            if filename_attr is None:
                continue
            basename = os.path.basename(filename_attr).lower()
            if basename in replaced:
                # Build new filename: keep the original directory prefix, swap
                # the basename with the _approx version.
                original_dir = filename_attr[: filename_attr.rfind("/") + 1] if "/" in filename_attr else ""
                new_basename = os.path.basename(replaced[basename])
                mesh_elem.set("filename", original_dir + new_basename)

        # Save to a temp file (preserve original)
        base, ext = os.path.splitext(self._urdf_file)
        out_path = f"{base}_temp{ext}"
        tree.write(out_path, encoding="unicode", xml_declaration=True)

        self.decomposition_applied.emit(out_path)
        self.accept()

    # ------------------------------------------------------------------
    # Button routing
    # ------------------------------------------------------------------
    def _on_action_clicked(self):
        if self._state == STATE_CONFIGURE:
            self._start_decomposition()
        elif self._state == STATE_RESULTS:
            self._apply_to_urdf()

    def _on_cancel_clicked(self):
        if self._state == STATE_PROCESSING:
            self._cancel_decomposition()
        else:
            self.reject()
