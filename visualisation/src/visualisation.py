import math
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore

def quaternion_to_euler(w, x, y, z):
    """
    Convert a quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw) in degrees.
    """
    # Roll (rotation about x-axis)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.degrees(np.arctan2(t0, t1))

    # Pitch (rotation about y-axis)
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.degrees(np.arcsin(t2))

    # Yaw (rotation about z-axis)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.degrees(np.arctan2(t3, t4))

    return roll, pitch, yaw

class AircraftVisualizer(gl.GLViewWidget):
    """
    AircraftVisualizer renders a 3D filled aircraft mesh and a ground plane.
    The aircraft orientation is updated based on quaternion data.
    
    Use set_axis_inversion(...) to flip the sign of roll/pitch/yaw if needed.
    """
    def __init__(self, data_store, parent=None):
        super(AircraftVisualizer, self).__init__(parent)
        self.data_store = data_store
        self.setWindowTitle('Aircraft 3D Visualization')

        # Axis inversion factors for roll, pitch, yaw. Set to +1 or -1.
        self.axis_inversion = {
            'roll': 1.0,
            'pitch': 1.0,
            'yaw': 1.0
        }

        # Adjust camera for a nicer default view.
        self.setCameraPosition(distance=25, elevation=15, azimuth=30)

        self.init_ground()
        self.init_aircraft()

        # Timer for updating the visualization (20 Hz)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_visualization)
        self.timer.start(50)

    def set_axis_inversion(self, roll_factor=1.0, pitch_factor=1.0, yaw_factor=1.0):
        """
        Set each factor to +1 or -1 to flip that axis if needed.
        Example usage: visualizer.set_axis_inversion(-1, 1, 1)
        """
        self.axis_inversion['roll'] = roll_factor
        self.axis_inversion['pitch'] = pitch_factor
        self.axis_inversion['yaw'] = yaw_factor

    def init_ground(self):
        """
        Create and add a simple ground plane.
        """
        grid = gl.GLGridItem()
        grid.setSize(50, 50)
        grid.setSpacing(1, 1)
        self.addItem(grid)

    def get_aircraft_mesh_data(self):
        """
        Generate mesh data for a filled aircraft.
        This creates a fuselage built from cross-sections and adds wings and stabilizers.
        Returns:
            all_vertices: (N, 3) numpy array of vertex positions.
            all_faces: (M, 3) numpy array of triangle indices.
            face_colors: (M, 4) numpy array with RGBA values for each face.
        """
        # --- Fuselage ---
        # Cross-section parameters (from tail to nose)
        xs = [-1.0, -0.5, 0.0, 0.5, 1.0]
        rys = [0.03, 0.05, 0.1, 0.1, 0.02]
        rzs = [0.03, 0.05, 0.1, 0.1, 0.02]
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

        # Set a metallic gray color for all faces.
        face_colors = np.tile(np.array([[0.75, 0.75, 0.75, 1.0]]), (all_faces.shape[0], 1))

        return all_vertices, all_faces, face_colors

    def init_aircraft(self):
        """
        Create a filled aircraft mesh and add it to the scene.
        """
        vertices, faces, face_colors = self.get_aircraft_mesh_data()
        # Save original vertices for rotation.
        self._orig_mesh_vertices = vertices.copy()
        self._faces = faces
        self._face_colors = face_colors

        self.aircraft_mesh = gl.GLMeshItem(
            vertexes=vertices,
            faces=faces,
            faceColors=face_colors,
            smooth=False,
            shader='shaded',
            drawEdges=True,
            edgeColor=(0, 0, 0, 1)
        )
        self.addItem(self.aircraft_mesh)

    def update_visualization(self):
        """
        Update the aircraft orientation using the latest quaternion data.
        """
        q = self.data_store.data["quaternion"]
        if None not in (q["w"], q["x"], q["y"], q["z"]):
            roll, pitch, yaw = quaternion_to_euler(q["w"], q["x"], q["y"], q["z"])

            # Apply any axis inversion factors.
            roll *= self.axis_inversion['roll']
            pitch *= self.axis_inversion['pitch']
            yaw *= self.axis_inversion['yaw']

            # Create composite rotation matrix from Euler angles (ZYX order).
            cr = np.cos(np.radians(roll))
            sr = np.sin(np.radians(roll))
            cp = np.cos(np.radians(pitch))
            sp = np.sin(np.radians(pitch))
            cy = np.cos(np.radians(yaw))
            sy = np.sin(np.radians(yaw))

            Rz = np.array([
                [cy, -sy, 0],
                [sy,  cy, 0],
                [0,    0, 1]
            ])
            Ry = np.array([
                [cp, 0, sp],
                [0, 1, 0],
                [-sp, 0, cp]
            ])
            Rx = np.array([
                [1, 0, 0],
                [0, cr, -sr],
                [0, sr, cr]
            ])
            R = Rz @ Ry @ Rx

            # Rotate the original mesh vertices.
            rotated_vertices = (R @ self._orig_mesh_vertices.T).T
            self.aircraft_mesh.setMeshData(
                vertexes=rotated_vertices,
                faces=self._faces,
                faceColors=self._face_colors,
                smooth=False,
                shader='shaded',
                drawEdges=True,
                edgeColor=(0, 0, 0, 1)
            )
