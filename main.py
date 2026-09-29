import pygame
import sys
import serial
import time
import costmap, caculation, draw, read_arduino
import math
from planner import Plan

pygame.init()

wheel_perimeter = 0.0675*math.pi;  
wheel_radius = 0.0675/2;
wheel_base = 0.195;

# Arduino Serial Port 
COM_PORT = 'COM5'  
input_array = []
input_trigger = False
BAUD_RATE = 115200
obstacle_history = []
car_theta = 0
car_x, car_y = 0, 0

WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 600
CENTER_X = WINDOW_WIDTH // 2
CENTER_Y = WINDOW_HEIGHT // 2
GRID_SIZE = 8

#state = (car_theta, car_x, car_y, goal_x, goal_y, path_cells)

screen_size = (WINDOW_WIDTH, WINDOW_HEIGHT, CENTER_X, CENTER_Y, GRID_SIZE)

SENSOR_OFFSET = 11.5 #cm
infla_radius = 6  

# color
COLOR_CROSSHAIR = (0, 150, 255, 100) 

screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Pygame 4-Quadrant Gridmap (Float Coordinates)")
clock = pygame.time.Clock()

font = pygame.font.SysFont("arial", 12)
font_bold = pygame.font.SysFont("arial", 14, bold=True)

base_map = draw.Gridmap(screen_size, GRID_SIZE)
pg_obs_map = draw.Obsmap(screen_size, GRID_SIZE)
pg_infla_map = draw.Inflamap(screen_size, GRID_SIZE)

COSTMAP = costmap.Costmap()

planner = Plan(COSTMAP.infla_layer.infla_map)


goal_x, goal_y = 0, 0
path_cells = []      
obstacle_distance, deltaSL, deltaSR = 800, 0, 0
base_linear_RPM = 0

need_replan = False
has_goal = False
print_counter = 0

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print(f"成功連結 {COM_PORT}")
    running = True
    time.sleep(2) 

    last_plan_time = time.time()
    while running:
    
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                
            if event.type == pygame.MOUSEBUTTONUP:
                last_grid_pos = None
                
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 2: 
                    mx, my = pygame.mouse.get_pos()
                    goal_x, goal_y = caculation.pixel_to_world(CENTER_X, CENTER_Y, mx, my, GRID_SIZE)
                    grid_goal_x, grid_goal_y = caculation.world_to_grid(goal_x, goal_y)
                    map_updated = True
                    has_goal = True

        if ser.in_waiting > 100:
            ser.reset_input_buffer()

        read_counter = 0
        map_updated = False
        tt_obs_points = [] 

        while ser.in_waiting > 0 and read_counter < 5:
            read_counter += 1
            print_counter += 1
            raw_data = ser.readline()
            try:
                data_string = raw_data.decode('utf-8', errors='ignore').strip()
                if not data_string:
                    continue


                obstacle_distance, deltaSL, deltaSR = read_arduino.decode(data_string)

                    
                if obstacle_distance is None :
                    continue

                car_x, car_y, car_theta = caculation.car_position(car_x, car_y, car_theta, deltaSL, deltaSR)

                grid_car_x, grid_car_y = caculation.world_to_grid(car_x, car_y)

                if obstacle_distance > 0:
                    rel_obs_x, rel_obs_y = caculation.obs_pos(obstacle_distance, car_theta)
                    obs_x = int(car_x) + int(rel_obs_x)
                    obs_y = int(car_y) + int(rel_obs_y)

                    grid_obs_x, grid_obs_y = caculation.world_to_grid(obs_x, obs_y)

                    # realtime obstacle
                    px, py = caculation.world_to_pixel(CENTER_X, CENTER_Y, obs_x, obs_y, GRID_SIZE)
                    obstacle_history.append((px, py))
                
                    change_points, removed_obs = COSTMAP.obs_layer.do_obs_layer(grid_car_x, grid_car_y, grid_obs_x, grid_obs_y)

                    COSTMAP.total_obs_points(infla_radius, change_points, removed_obs, tt_obs_points)

                    pg_infla_map.redraw_del_infla_points(infla_radius, removed_obs)
                    pg_obs_map.draw_space_obstacle(change_points)
                    if tt_obs_points:
                        unique_obs_points = list(set(tt_obs_points))
                        infla_layer = COSTMAP.infla_layer.do_infla_layer(unique_obs_points, infla_radius)
                        pg_infla_map.draw_infla_points(infla_layer)

                    if len(obstacle_history) > 500:
                        obstacle_history.pop(0)

                map_updated = True
            except UnicodeDecodeError:
                pass

        current_time = time.time()
        #檢查路徑、重新規劃
        if map_updated and path_cells:
            if current_time - last_plan_time > planner.MIN_REPLAN_INTERVAL:
                if planner.is_path_blocked(path_cells, COSTMAP.infla_layer.infla_map):
                    need_replan = True

        if current_time - last_plan_time > planner.REPLAN_TIMEOUT:
            need_replan = True

        #偵測終點
        if has_goal :
            d_to_final_goal = math.hypot(goal_x - car_x, goal_y - car_y)
            if d_to_final_goal < 10:
                has_goal = False
                need_replan = False
                path_cells = []
                send_string = f"{0},{0}\n"
                ser.write(send_string.encode('utf-8'))
                print("已到達目標點，停止小車。")

        #A*
        if has_goal and need_replan:
            new_path = planner.astar((grid_car_x, grid_car_y), (grid_goal_x, grid_goal_y))

            if new_path and len(new_path) > 1:
                path_cells = new_path

            need_replan = False
            last_plan_time = current_time
            #print(path_cells)

        if has_goal:
            d_to_final_goal = math.hypot(goal_x - car_x, goal_y - car_y)
            if d_to_final_goal < 6.0:  # 距離終點小於 6cm 視為到達
                has_goal = False
                need_replan = False
                path_cells = []
                ser.write(b"0,0\n")
                print(">>> 已成功到達終點，煞停小車！")

        #線性、旋轉 RPM
        if has_goal and path_cells:
            next_target = path_cells[1]  if len(path_cells) > 1 else path_cells[0]

            errordistance, errorAngle = caculation.error_calculation(car_x, car_y, car_theta, next_target[0], next_target[1])

            if abs(errordistance) < 8 and len(path_cells) > 2:
                path_cells.pop(0)
                next_target = path_cells[1]  
                next_target = path_cells[1] if len(path_cells) > 1 else path_cells[0]
                errordistance, errorAngle = caculation.error_calculation(car_x, car_y, car_theta, next_target[0], next_target[1])
                #RPM calculation

            if abs(errordistance) > 2:
                target_v = 1.7 * errordistance/100.0
                target_v = max(0.06, min(target_v, 0.24))  
                base_linear_RPM = (target_v / wheel_perimeter) * 60.0

                if abs(errorAngle) > 2:
                    linear_factor = 1.0 - pow((min(abs(errorAngle), 180.0) / 180.0),2)
                    linear_RPM = base_linear_RPM * linear_factor
                    target_w = 4.3*errorAngle
                    target_w = max(-90, min(target_w, 90))
                    rotate_RPM = target_w * wheel_base / (wheel_radius * 12)
                else:
                    rotate_RPM = 0
                    linear_RPM = base_linear_RPM
            else:
                linear_RPM = 0
                rotate_RPM = 0  

            send_string = f"{int(linear_RPM)},{int(rotate_RPM)}\n"
            ser.write(send_string.encode('utf-8'))

        else:
            ser.write(b"0,0\n")

        #base map
        screen.fill(base_map.COLOR[0])  # Fill the background with the base color
        base_map.draw_grid(screen)
        base_map.draw_Unstd_obs_point(obstacle_history)
        base_map.draw_car_position(car_x, car_y, car_theta)
        base_map.draw_Astar_path(path_cells)
        base_map.draw_goal_point(goal_x, goal_y)

        #screen.blit(pg_obs_map.obs_map_surface, (0, 0))
        screen.blit(pg_infla_map.infla_map_surface, (0, 0))

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()
    sys.exit()

except serial.SerialException as e:
    print(f"無法開啟序列埠 {COM_PORT}，原因: {e}")
    print("提示：請確認 VS Code 的 Serial Monitor 已經關閉！")
except KeyboardInterrupt:
    print("\n程式被使用者中止。")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("序列埠已關閉。")