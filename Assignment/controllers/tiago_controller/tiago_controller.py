from controller import Supervisor, Keyboard
from knowledge.kg import KG 
from perception.detect import CameraDetection
from perception.lidar import Lidar
import numpy as np
from skills.grasp import GraspSkill
from skills.head_follow import HeadFollowSkill
from skills.approach import ApproachSkill
from skills.navigate import NavigateSkill          

def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    # --- Sensor- en Apparaat-Setup ---
    kb = robot.getKeyboard()
    kb.enable(timestep)

    cam = CameraDetection(robot, camera_name="Astra rgb", model_path="yolo11n.pt")
    cam.enable(timestep)

    lidar = Lidar(robot, lidar_def="Hokuyo URG-04LX-UG01", lidar_offset=(0.202, 0.0))
    lidar.enable(timestep)

    # --- Lidar hoogtecorrectie ---
    tiago_node = robot.getFromDef("TIAGo")
    if tiago_node:
        lidar_slot_field = tiago_node.getField("lidarSlot")
        if lidar_slot_field and lidar_slot_field.getCount() > 0:
            lidar_node = lidar_slot_field.getMFNode(0)
            if lidar_node:
                translation_field = lidar_node.getField("translation")
                x, y, z = translation_field.getSFVec3f()
                translation_field.setSFVec3f([x, y, z + 0.29])

    # --- Motors (Opvragen voor handmatige controle) ---
    left_motor = robot.getDevice("wheel_left_joint")
    right_motor = robot.getDevice("wheel_right_joint")

    # --- KG SETUP (NIEUW) ---
    kg = KG()
    
    # Definieer de start- en doelposities in de KG
    kg.add_waypoint(symbol="start", def_name="WP_START")     # Startpunt in Webots
    kg.add_waypoint(symbol="crates", def_name="WP_CRATES")   # Doelpunt in Webots
    
    # Assert de startpositie van de robot in de KG
    kg.assert_robot_at(robot="tiago", place="start") 

    # --- SKILL INITIALISATIE ---
    grasp = GraspSkill(robot, cam)
    head_follow = HeadFollowSkill(robot, cam)
    approach = ApproachSkill(robot, cam, target_distance=0.4)
    
    # Gebruik de NavigateSkill (met KG)
    navigate = NavigateSkill(robot, kg, cam) 
    navigate.setup()
    
    # De PureNavigateSkill is nu overbodig
    # pure_nav = PureNavigateSkill(robot)
    # pure_nav.setup()

    print("Keyboard control actief.")

    # --- Control Variabelen ---
    v_forward = 5.0
    v_turn = 3.0
    navigating = False 
    doel_plek = "crates" # Het symbolische doel

    print(f"Druk op 'N' om autonome navigatie te starten van 'start' naar '{doel_plek}'.")

    # === WEBOTS SIMULATIE LUS ===
    while robot.step(timestep) != -1:
        approaching = False # Plaats dit buiten de lus als het door een skill wordt beheerd

        cam.get_image()
        lidar.update_global_map()
        key = kb.getKey()

        # === TOETSENBORD COMMANDO'S ===
        if key == ord('G'):
            grasp.execute("apple")
            
        # Start/Stop de autonome navigatie
        if key == ord('N'):
            if not navigating:
                try:
                    # De NavigateSkill haalt coördinaten op via de KG
                    navigate.start_place(doel_plek) 
                    navigating = True
                    print(f"Navigatie gestart naar '{doel_plek}'...")
                except RuntimeError as e:
                    print(f"ERROR: Kan navigatie naar {doel_plek} niet starten: {e}")
            else:
                navigating = False
                print("Navigatie handmatig gestopt.")
        
        # === MOTOR CONTROL PRIORITEIT ===
        
        # 1. AUTONOME CONTROLE (Hoogste prioriteit)
        if navigating:
            # Gebruik de NavigateSkill met KG
            status = navigate.step() 
            
            if status == "done":
                print(f"Bestemming '{doel_plek}' bereikt!")
                navigating = False
                # Update de KG met de nieuwe robotpositie
                kg.assert_robot_at(robot="tiago", place=doel_plek)
                left_motor.setVelocity(0.0) 
                right_motor.setVelocity(0.0)
            elif status == "failed":
                print("Navigatie mislukt!")
                navigating = False
                left_motor.setVelocity(0.0) 
                right_motor.setVelocity(0.0)
        
        # 2. HANDMATIGE KEYBOARD CONTROLE of NIET-BEWEGEN
        # Dit blok wordt alleen uitgevoerd als 'navigating' = False en 'approaching' = False
        elif not approaching: 
            v_l = 0.0
            v_r = 0.0

            if key == Keyboard.UP:
                v_l = v_forward
                v_r = v_forward
            elif key == Keyboard.DOWN:
                v_l = -v_forward
                v_r = -v_forward
            elif key == Keyboard.LEFT:
                v_l = -v_turn
                v_r = v_turn
            elif key == Keyboard.RIGHT:
                v_l = v_turn
                v_r = -v_turn
            
            # Voer de handmatige/nul snelheden uit
            left_motor.setVelocity(v_l)
            right_motor.setVelocity(v_r)
        
        # OPMERKING: De dubbele elif hierboven is verwijderd.
        # Als navigating=False en approaching=True, gebeurt hier niets (wat correct is, 
        # want ApproachSkill zou de motoren moeten aansturen).


if __name__ == "__main__":
    main()