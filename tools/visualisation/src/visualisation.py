import math
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore

def quaternion_to_euler(w, x, y, z):
    """
    Convert a quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw) in degrees.
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.degrees(np.arcsin(t2))

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

        # Adjust camera: closer distance, keeping a good elevation and azimuth.
        self.setCameraPosition(distance=5, elevation=15, azimuth=30)

        self.init_ground()
        self.init_aircraft()
        self.init_control_surfaces()

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

    def init_control_surfaces(self):
        """
        Create separate mesh items for the control surfaces:
        left/right ailerons, elevator, and rudder.
        Their geometry is defined in the aircraft coordinate system.
        """
        # Left Aileron (moved to the trailing edge of the left wing)
        left_aileron_vertices = np.array([
            [-0.1, -0.6, 0.0],
            [-0.1, -1.2, 0.0],
            [0.0,  -1.2, 0.0],
            [0.0,  -0.6, 0.0]
        ])
        left_aileron_faces = np.array([[0, 1, 2], [0, 2, 3]])
        left_aileron_colors = np.array([[0.8, 0.0, 0.0, 1.0]] * left_aileron_faces.shape[0])
        self.left_aileron = gl.GLMeshItem(
            vertexes=left_aileron_vertices,
            faces=left_aileron_faces,
            faceColors=left_aileron_colors,
            smooth=False,
            shader='shaded',
            drawEdges=True,
            edgeColor=(0, 0, 0, 1)
        )
        self.addItem(self.left_aileron)
        self._orig_left_aileron = left_aileron_vertices.copy()
        self._left_aileron_faces = left_aileron_faces
        self._left_aileron_colors = left_aileron_colors

        # Right Aileron (moved to the trailing edge of the right wing)
        right_aileron_vertices = np.array([
            [-0.1,  0.6, 0.0],
            [-0.1,  1.2, 0.0],
            [0.0,   1.2, 0.0],
            [0.0,   0.6, 0.0]
        ])
        right_aileron_faces = np.array([[0, 1, 2], [0, 2, 3]])
        right_aileron_colors = np.array([[0.8, 0.0, 0.0, 1.0]] * right_aileron_faces.shape[0])
        self.right_aileron = gl.GLMeshItem(
            vertexes=right_aileron_vertices,
            faces=right_aileron_faces,
            faceColors=right_aileron_colors,
            smooth=False,
            shader='shaded',
            drawEdges=True,
            edgeColor=(0, 0, 0, 1)
        )
        self.addItem(self.right_aileron)
        self._orig_right_aileron = right_aileron_vertices.copy()
        self._right_aileron_faces = right_aileron_faces
        self._right_aileron_colors = right_aileron_colors

        # Elevator (attached to the trailing edge of the horizontal stabilizer) 
        elevator_vertices = np.array([ 
            [-1.1, -0.4, 0.0], 
            [-1.1, 0.4, 0.0], 
            [-1.0, 0.4, 0.0], 
            [-1.0, -0.4, 0.0] 
        ]) 
        
        elevator_faces = np.array([[0, 1, 2], [0, 2, 3]]) 
        elevator_colors = np.array([[0.0, 0.8, 0.0, 1.0]] * elevator_faces.shape[0]) 
        self.elevator = gl.GLMeshItem( 
            vertexes=elevator_vertices, 
            faces=elevator_faces, 
            faceColors=elevator_colors, 
            smooth=False, 
            shader='shaded', 
            drawEdges=True, 
            edgeColor=(0, 0, 0, 1) 
        ) 
        self.addItem(self.elevator) 
        self._orig_elevator = elevator_vertices.copy() 
        self._elevator_faces = elevator_faces 
        self._elevator_colors = elevator_colors

        # Rudder (attached to the trailing edge of the vertical fin)
        rudder_vertices = np.array([
            [-1.0, 0.0, -0.6],
            [-1.0, 0.0, -0.3],
            [-1.1,  0.0, -0.3],
            [-1.1,  0.0, -0.6]
        ])
        rudder_faces = np.array([[0, 1, 2], [0, 2, 3]])
        rudder_colors = np.array([[0.0, 0.0, 0.8, 1.0]] * rudder_faces.shape[0])
        self.rudder = gl.GLMeshItem(
            vertexes=rudder_vertices,
            faces=rudder_faces,
            faceColors=rudder_colors,
            smooth=False,
            shader='shaded',
            drawEdges=True,
            edgeColor=(0, 0, 0, 1)
        )
        self.addItem(self.rudder)
        self._orig_rudder = rudder_vertices.copy()
        self._rudder_faces = rudder_faces
        self._rudder_colors = rudder_colors

    @staticmethod
    def rotation_matrix(axis, angle):
        """
        Compute a rotation matrix for rotating 'angle' radians around 'axis'
        """
        axis = np.array(axis)
        axis = axis / np.linalg.norm(axis)
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        ux, uy, uz = axis
        return np.array([
            [cos_a + ux*ux*(1-cos_a),      ux*uy*(1-cos_a) - uz*sin_a, ux*uz*(1-cos_a) + uy*sin_a],
            [uy*ux*(1-cos_a) + uz*sin_a, cos_a + uy*uy*(1-cos_a),      uy*uz*(1-cos_a) - ux*sin_a],
            [uz*ux*(1-cos_a) - uy*sin_a, uz*uy*(1-cos_a) + ux*sin_a, cos_a + uz*uz*(1-cos_a)]
        ])

    def update_visualization(self):
        """
        Update the aircraft orientation using the latest quaternion data.
        Also update the control surfaces based on the command values.
        Applies a final z‑flip so that -z is up and +z is down.
        """
        q = self.data_store.data["quaternion"]
        if None not in (q["w"], q["x"], q["y"], q["z"]):
            roll, pitch, yaw = quaternion_to_euler(q["w"], q["x"], q["y"], q["z"])

            # Apply axis inversion factors.
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
            # Flip the z-axis so that -z is up and +z is down.
            rotated_vertices[:, 2] *= -1

            self.aircraft_mesh.setMeshData(
                vertexes=rotated_vertices,
                faces=self._faces,
                faceColors=self._face_colors,
                smooth=False,
                shader='shaded',
                drawEdges=True,
                edgeColor=(0, 0, 0, 1)
            )

            # Helper function: apply deflection rotation about a hinge,
            # then overall aircraft rotation R and final z flip.
            def transform_surface(orig_vertices, hinge, axis, deflection_angle):
                relative = orig_vertices - hinge
                R_defl = AircraftVisualizer.rotation_matrix(axis, deflection_angle)
                rotated_relative = (R_defl @ relative.T).T
                transformed = rotated_relative + hinge
                transformed = (R @ transformed.T).T
                transformed[:, 2] *= -1
                return transformed

            # Get command values (default to 0 if not available)
            # Commands are in the range -1 to 1; scale them so that ±1 is 90° deflection.
            rate_controller_roll = self.data_store.data["rate"]["roll"] or 0.0
            rate_controller_pitch = self.data_store.data["rate"]["pitch"] or 0.0
            rate_controller_yaw = self.data_store.data["rate"]["yaw"] or 0.0

            max_deflection = np.radians(90)  # 90° maximum for all control surfaces

            # Compute actual deflection angles.
            left_aileron_angle = -rate_controller_roll * max_deflection
            right_aileron_angle = rate_controller_roll * max_deflection
            elevator_angle = -rate_controller_pitch * max_deflection
            rudder_angle = rate_controller_yaw * max_deflection

            # Update left aileron.
            # Hinge at the trailing edge of the left wing (x = 0.0).
            hinge_left = np.array([0.0, -0.6, 0.0])
            transformed_left = transform_surface(self._orig_left_aileron, hinge_left, [0, 1, 0], left_aileron_angle)
            self.left_aileron.setMeshData(
                vertexes=transformed_left,
                faces=self._left_aileron_faces,
                faceColors=self._left_aileron_colors,
                smooth=False,
                shader='shaded',
                drawEdges=True,
                edgeColor=(0, 0, 0, 1)
            )

            # Update right aileron.
            hinge_right = np.array([0.0, 0.6, 0.0])
            transformed_right = transform_surface(self._orig_right_aileron, hinge_right, [0, 1, 0], right_aileron_angle)
            self.right_aileron.setMeshData(
                vertexes=transformed_right,
                faces=self._right_aileron_faces,
                faceColors=self._right_aileron_colors,
                smooth=False,
                shader='shaded',
                drawEdges=True,
                edgeColor=(0, 0, 0, 1)
            )

            # Update elevator.
            hinge_elevator = np.array([-1.0, 0.0, 0.0]) 
            transformed_elevator = transform_surface(self._orig_elevator, hinge_elevator, [0, 1, 0], elevator_angle) 
            self.elevator.setMeshData( 
                vertexes=transformed_elevator, 
                faces=self._elevator_faces, 
                faceColors=self._elevator_colors, 
                smooth=False, 
                shader='shaded', 
                drawEdges=True, 
                edgeColor=(0, 0, 0, 1) 
            )

            # Update rudder.
            hinge_rudder = np.array([-1.0, 0.0, -0.6])
            transformed_rudder = transform_surface(self._orig_rudder, hinge_rudder, [0, 0, 1], rudder_angle)
            self.rudder.setMeshData(
                vertexes=transformed_rudder,
                faces=self._rudder_faces,
                faceColors=self._rudder_colors,
                smooth=False,
                shader='shaded',
                drawEdges=True,
                edgeColor=(0, 0, 0, 1)
            )
