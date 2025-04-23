import math
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore
from PyQt5.QtGui import QMatrix4x4

def quat_to_matrix(w, x, y, z):
    """
    Convert normalized quaternion to a 3x3 rotation matrix.
    If quaternion is zero-length (e.g. uninitialised), return identity.
    """
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n == 0.0:
        return np.eye(3)
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [2*(x*y + z*w),   1-2*(x*x+z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x*x+y*y)]
    ])

class AircraftVisualizer(gl.GLViewWidget):
    """
    AircraftVisualizer renders a 3D filled aircraft mesh and a ground plane.
    The aircraft orientation is updated based on quaternion data and control surface commands.
    """
    def __init__(self, data_store, parent=None):
        super().__init__(parent)
        self.data_store = data_store
        self.setWindowTitle('Aircraft 3D Visualization')
        self.axis_inversion = {'roll':1.0,'pitch':1.0,'yaw':1.0}
        self.setCameraPosition(distance=5, elevation=15, azimuth=30)

        self.init_ground()
        self.init_aircraft()
        self.init_control_surfaces()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_visualization)
        self.timer.start(50)

    def init_ground(self):
        grid = gl.GLGridItem()
        grid.setSize(50,50)
        grid.setSpacing(1,1)
        self.addItem(grid)

    def get_aircraft_mesh_data(self):
        """
        Generate mesh data for a filled aircraft.
        Creates a fuselage from elliptical cross-sections and adds wings and stabilizers.
        Returns:
            all_vertices: (N, 3) numpy array of vertex positions.
            all_faces: (M, 3) numpy array of triangle indices.
            face_colors: (M, 4) numpy array with RGBA values for each face.
        """
        # --- Fuselage ---
        xs = [-1.0, -0.5, 0.0, 0.5, 1.0]
        rys = [0.03, 0.05, 0.1, 0.1, 0.02]
        rzs = [-0.03, -0.05, -0.1, -0.1, -0.02]
        num_segments = 16

        fuselage_vertices = []
        for i, x in enumerate(xs):
            for j in range(num_segments):
                angle = 2 * math.pi * j / num_segments
                y = rys[i] * math.cos(angle)
                z = rzs[i] * math.sin(angle)
                fuselage_vertices.append([x, y, z])
        fuselage_vertices = np.array(fuselage_vertices)

        fuselage_faces = []
        for i in range(len(xs)-1):
            for j in range(num_segments):
                current = i * num_segments + j
                next = i * num_segments + ((j+1) % num_segments)
                current2 = (i+1) * num_segments + j
                next2 = (i+1) * num_segments + ((j+1) % num_segments)
                fuselage_faces.append([current, next, current2])
                fuselage_faces.append([next, next2, current2])
        fuselage_faces = np.array(fuselage_faces)

        # --- Left Wing ---
        left_wing_vertices = np.array([
            [0.0,  0.0, 0.0],
            [0.0, -1.2, 0.0],
            [0.2, -1.2, 0.0],
            [0.2,  0.0, 0.0]
        ])
        left_wing_faces = np.array([
            [0, 1, 2],
            [0, 2, 3]
        ])

        # --- Right Wing ---
        right_wing_vertices = np.array([
            [0.0,  0.0, 0.0],
            [0.0,  1.2, 0.0],
            [0.2,  1.2, 0.0],
            [0.2,  0.0, 0.0]
        ])
        right_wing_faces = np.array([
            [0, 1, 2],
            [0, 2, 3]
        ])

        # --- Horizontal Stabilizer (Tail Wing) ---
        horizontal_stab_vertices = np.array([
            [-1.0, -0.4, 0.0],
            [-0.8, -0.4, 0.0],
            [-0.8,  0.4, 0.0],
            [-1.0,  0.4, 0.0]
        ])
        horizontal_stab_faces = np.array([
            [0, 1, 2],
            [0, 2, 3]
        ])

        # --- Vertical Fin ---
        vertical_fin_vertices = np.array([
            [-1.0, 0.0, 0.0],
            [-1.0, 0.0, -0.6],
            [-0.7, 0.0, 0.0]
        ])
        vertical_fin_faces = np.array([
            [0, 1, 2]
        ])

        # --- Combine all parts ---
        vertices_list = [
            fuselage_vertices,
            left_wing_vertices,
            right_wing_vertices,
            horizontal_stab_vertices,
            vertical_fin_vertices
        ]
        faces_list = [
            fuselage_faces,
            left_wing_faces,
            right_wing_faces,
            horizontal_stab_faces,
            vertical_fin_faces
        ]

        offset = 0
        all_vertices = []
        all_faces = []
        for verts, faces in zip(vertices_list, faces_list):
            all_vertices.append(verts)
            faces_offset = faces + offset
            all_faces.append(faces_offset)
            offset += verts.shape[0]
        all_vertices = np.vstack(all_vertices)
        all_faces = np.vstack(all_faces)

        # Use a very light metallic color for all faces (almost white).
        face_colors = np.tile(np.array([[0.95, 0.95, 0.95, 1.0]]), (all_faces.shape[0], 1))

        return all_vertices, all_faces, face_colors

    def init_aircraft(self):
        vertices, faces, colors = self.get_aircraft_mesh_data()
        self.aircraft_mesh = gl.GLMeshItem(
            vertexes=vertices, faces=faces, faceColors=colors,
            smooth=False, shader='shaded', drawEdges=True, edgeColor=(0,0,0,1)
        )
        self.aircraft_mesh.translate(0,0,0)
        self.addItem(self.aircraft_mesh)

    def init_control_surfaces(self):
        """
        Create mesh items for the control surfaces (ailerons, elevator, rudder)
        and record their hinge points for use in update_visualization().
        """
        # Dictionary mapping surface name → hinge point in aircraft coords
        self.hinges = {}

        # --- Left Aileron ---
        left_verts = np.array([
            [-0.1, -0.6, 0.0],
            [-0.1, -1.2, 0.0],
            [ 0.0, -1.2, 0.0],
            [ 0.0, -0.6, 0.0]
        ])
        left_faces = np.array([[0, 1, 2], [0, 2, 3]])
        left_colors = np.array([[0.8, 0.0, 0.0, 1.0]] * left_faces.shape[0])
        self.left_aileron = gl.GLMeshItem(
            vertexes=left_verts,
            faces=left_faces,
            faceColors=left_colors,
            smooth=False,
            shader='shaded',
            drawEdges=True,
            edgeColor=(0, 0, 0, 1)
        )
        self.addItem(self.left_aileron)
        # Hinge at trailing edge midpoint
        self.hinges['left_aileron'] = np.array([0.0, -0.6, 0.0])

        # --- Right Aileron ---
        right_verts = np.array([
            [-0.1,  0.6, 0.0],
            [-0.1,  1.2, 0.0],
            [ 0.0,  1.2, 0.0],
            [ 0.0,  0.6, 0.0]
        ])
        right_faces = np.array([[0, 1, 2], [0, 2, 3]])
        right_colors = np.array([[0.8, 0.0, 0.0, 1.0]] * right_faces.shape[0])
        self.right_aileron = gl.GLMeshItem(
            vertexes=right_verts,
            faces=right_faces,
            faceColors=right_colors,
            smooth=False,
            shader='shaded',
            drawEdges=True,
            edgeColor=(0, 0, 0, 1)
        )
        self.addItem(self.right_aileron)
        self.hinges['right_aileron'] = np.array([0.0,  0.6, 0.0])

        # --- Elevator ---
        elev_verts = np.array([
            [-1.1, -0.4, 0.0],
            [-1.1,  0.4, 0.0],
            [-1.0,  0.4, 0.0],
            [-1.0, -0.4, 0.0]
        ])
        elev_faces = np.array([[0, 1, 2], [0, 2, 3]])
        elev_colors = np.array([[0.0, 0.8, 0.0, 1.0]] * elev_faces.shape[0])
        self.elevator = gl.GLMeshItem(
            vertexes=elev_verts,
            faces=elev_faces,
            faceColors=elev_colors,
            smooth=False,
            shader='shaded',
            drawEdges=True,
            edgeColor=(0, 0, 0, 1)
        )
        self.addItem(self.elevator)
        # Hinge at tailplane leading edge
        self.hinges['elevator'] = np.array([-1.0, 0.0, 0.0])

        # --- Rudder ---
        rudder_verts = np.array([
            [-1.0, 0.0, -0.6],
            [-1.0, 0.0, -0.3],
            [-1.1, 0.0, -0.3],
            [-1.1, 0.0, -0.6]
        ])
        rudder_faces = np.array([[0, 1, 2], [0, 2, 3]])
        rudder_colors = np.array([[0.0, 0.0, 0.8, 1.0]] * rudder_faces.shape[0])
        self.rudder = gl.GLMeshItem(
            vertexes=rudder_verts,
            faces=rudder_faces,
            faceColors=rudder_colors,
            smooth=False,
            shader='shaded',
            drawEdges=True,
            edgeColor=(0, 0, 0, 1)
        )
        self.addItem(self.rudder)
        # Hinge at vertical fin trailing edge
        self.hinges['rudder'] = np.array([-1.0, 0.0, -0.6])

    def update_visualization(self):
        """
        Update the aircraft and control surfaces.
        Applies a 4×4 quaternion‐based transform to the fuselage mesh,
        then re‐applies hinge rotations + that same transform to each surface.
        """
        # 1) Grab quaternion
        w = self.data_store.get_latest("imu", "quaternion", "w")
        x = self.data_store.get_latest("imu", "quaternion", "x")
        y = self.data_store.get_latest("imu", "quaternion", "y")
        z = self.data_store.get_latest("imu", "quaternion", "z")
        if None in (w, x, y, z):
            return

        # 2) Build 3×3 rotation from quaternion
        M3 = quat_to_matrix(w, x, y, z)

        # 3) Build 4×4 homogeneous matrix for Qt
        M4 = np.eye(4, dtype=float)
        M4[:3, :3] = M3
        flat = M4.flatten().tolist()
        qt_mat = QMatrix4x4(*flat)

        # 4) Apply to fuselage mesh
        self.aircraft_mesh.resetTransform()
        self.aircraft_mesh.setTransform(qt_mat)

        # 5) Fetch rate‐loop commands for control‐surface deflections
        roll_cmd  = self.data_store.get_latest("controller", "rate", "roll")  or 0.0
        pitch_cmd = self.data_store.get_latest("controller", "rate", "pitch") or 0.0
        yaw_cmd   = self.data_store.get_latest("controller", "rate", "yaw")   or 0.0

        # 5) Compute deflection in degrees (±30° max)
        # ±30° max deflection
        max_defl = 30.0

        # Helper to set a single‐surface transform
        def set_surface(mesh, hinge, axis, angle_deg):
            mesh.resetTransform()
            # rotate about its hinge
            mesh.translate(*hinge)
            mesh.rotate(angle_deg, *axis)
            mesh.translate(*(-hinge))
            # then apply the global orientation
            mesh.setTransform(qt_mat)

        # 6) Update each surface
        set_surface(self.left_aileron,  self.hinges['left_aileron'],  (0,1,0), -roll_cmd  * max_defl)
        set_surface(self.right_aileron, self.hinges['right_aileron'], (0,1,0),  roll_cmd  * max_defl)
        set_surface(self.elevator,      self.hinges['elevator'],      (0,1,0), -pitch_cmd * max_defl)
        set_surface(self.rudder,        self.hinges['rudder'],        (0,0,1),  yaw_cmd   * max_defl)
