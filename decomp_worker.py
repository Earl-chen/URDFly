#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import trimesh
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal


class MeshInfo:
    """Data class holding per-mesh decomposition configuration and results."""

    def __init__(self, file_path, link_name, max_convex_hulls=32, enabled=True):
        # --- configuration (set before decomposition) ---
        self.file_path = file_path
        self.link_name = link_name
        self.file_name = os.path.basename(file_path)
        self.original_faces = 0        # populated when table is built
        self.max_convex_hulls = max_convex_hulls
        self.enabled = enabled

        # --- results (populated by worker) ---
        self.result_hull_count = 0
        self.result_faces = 0
        self.output_file = ""
        self.status = "pending"         # pending | success | fallback | error | skipped
        self.error_message = ""


class DecompWorker(QThread):
    """Background worker that performs convex decomposition for a list of meshes.

    Signals
    -------
    mesh_started(int)
        Emitted when processing of mesh at *index* begins.
    mesh_finished(int, object)
        Emitted when mesh at *index* finishes; the second arg is the MeshInfo.
    overall_progress(int, int)
        (current_1based, total) – convenience for progress-bar updates.
    all_finished(list)
        Emitted once all meshes have been processed; carries the full
        MeshInfo list (same objects that were passed in, now mutated).
    """

    mesh_started = pyqtSignal(int)
    mesh_finished = pyqtSignal(int, object)
    overall_progress = pyqtSignal(int, int)
    all_finished = pyqtSignal(list)

    def __init__(self, mesh_infos, parent=None):
        super().__init__(parent)
        self._mesh_infos = mesh_infos
        self._cancelled = False

    # ------------------------------------------------------------------
    # public helpers
    # ------------------------------------------------------------------
    def cancel(self):
        """Request cancellation.  The current mesh will still finish."""
        self._cancelled = True

    # ------------------------------------------------------------------
    # thread entry point
    # ------------------------------------------------------------------
    def run(self):
        enabled_indices = [i for i, m in enumerate(self._mesh_infos) if m.enabled]
        total = len(enabled_indices)
        done = 0

        for idx in range(len(self._mesh_infos)):
            info = self._mesh_infos[idx]

            if not info.enabled:
                info.status = "skipped"
                self.mesh_finished.emit(idx, info)
                continue

            if self._cancelled:
                info.status = "skipped"
                self.mesh_finished.emit(idx, info)
                continue

            self.mesh_started.emit(idx)
            done += 1
            self.overall_progress.emit(done, total)

            try:
                mesh = trimesh.load(info.file_path)
                info.original_faces = int(mesh.faces.shape[0])

                try:
                    convex_pieces = mesh.convex_decomposition(
                        maxConvexHulls=info.max_convex_hulls,
                    )

                    if isinstance(convex_pieces, list):
                        hull_count = len(convex_pieces)
                        vertices_list = []
                        faces_list = []
                        offset = 0
                        for piece in convex_pieces:
                            vertices_list.append(piece.vertices)
                            faces_list.append(piece.faces + offset)
                            offset += len(piece.vertices)
                        if vertices_list:
                            combined_v = np.vstack(vertices_list)
                            combined_f = np.vstack(faces_list)
                            result_mesh = trimesh.Trimesh(
                                vertices=combined_v, faces=combined_f
                            )
                        else:
                            raise RuntimeError("V-HACD produced no valid pieces")
                    else:
                        hull_count = 1
                        result_mesh = convex_pieces

                    info.result_hull_count = hull_count
                    info.result_faces = int(result_mesh.faces.shape[0])
                    info.status = "success"

                except Exception:
                    # Fallback: single convex hull
                    result_mesh = mesh.convex_hull
                    info.result_hull_count = 1
                    info.result_faces = int(result_mesh.faces.shape[0])
                    info.status = "fallback"

                # Write output file
                base, ext = os.path.splitext(info.file_path)
                info.output_file = f"{base}_approx{ext}"
                result_mesh.export(info.output_file)

            except Exception as e:
                info.status = "error"
                info.error_message = str(e)

            self.mesh_finished.emit(idx, info)

        self.all_finished.emit(self._mesh_infos)
