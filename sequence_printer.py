import math
from cams import cams
import copy

y = 0
low = 0
x_scale = 1000
y_spacing = 100
high = -50 # Negative is up in SVG
rise_time = 0.02
fall_time = 0.01
height = len(cams)*y_spacing+100
cam_low_radius = 75 # mm
cam_high_radius = 80
polar_pos_x = x_scale
print(f'<svg width="{x_scale}" height="{height}">')

def combine(steps):
    output = [steps.pop(0)]
    while len(steps)>0:
        (current_start, length) = output[-1]
        (next_start, next_end) = steps.pop(0)
        if current_start+length > next_start:
            output.pop()
            output.append((current_start, next_end-current_start))
        else:
            output.append((next_start, next_end))
    return output

openscad_angle_step = math.pi*2/50;

def make_openscad_cam(bumps):
    angle = 0
    index = 0
    next_bump = None if len(bumps)==index else bumps[index]
    points = []
    radius = cam_low_radius
    while angle <= math.pi*2:
        if next_bump:
            (start, length) = next_bump
        else:
            (start, length) = (999, 999)
        end = start+length
        start *= math.pi*2
        end *= math.pi*2
        points.append((radius*math.cos(angle), radius*math.sin(angle)))
        if angle > start and radius == cam_low_radius:
            angle += rise_time*math.pi*2
            radius = cam_high_radius
            points.append((radius*math.cos(angle), radius*math.sin(angle)))
        if angle > end and radius == cam_high_radius:
            angle += fall_time*math.pi*2
            radius = cam_low_radius
            points.append((radius*math.cos(angle), radius*math.sin(angle)))
            index += 1
            next_bump = None if len(bumps)==index else bumps[index]
        angle += openscad_angle_step
        print(start, end, angle, radius)
    return points

f = open("cams.scad", "w")
f.write(f"bolt_circle_diameter = 125;\n");
f.write(f"bolt_circle_radius = bolt_circle_diameter/2;\n");
f.write(f"kerf = 0.1;\n")
camno = 1
cam_instances = ""
for c in cams:
    polygon = [(0,y)]
    polar_polygon = [(polar_pos_x+cam_low_radius,0)]
    combined_steps = combine(c.steps)
    polar_data = f"M {polar_polygon[0][0]} {polar_polygon[0][1]} "
    last_pos = 0
    for (start, length) in combined_steps:
        end = start+length

        while start-last_pos > 0.1:
            last_pos += 0.1
            polar_data += f"A {cam_low_radius} {cam_low_radius} 0 0 1 {polar_pos_x+cam_low_radius*math.cos(last_pos*math.pi*2)} {cam_low_radius*math.sin(last_pos*math.pi*2)} "
        polar_data += f"A {cam_low_radius} {cam_low_radius} 0 {1 if (last_pos-start)>0.5 else 0} 1 {polar_pos_x+cam_low_radius*math.cos(start*math.pi*2)} {cam_low_radius*math.sin(start*math.pi*2)} "
        polar_data += f"L {polar_pos_x+cam_high_radius*math.cos((start+rise_time)*math.pi*2)}, {cam_high_radius*math.sin((start+rise_time)*math.pi*2)} "
        polar_data += f"A {cam_high_radius} {cam_high_radius} 0 0 1 {polar_pos_x+cam_high_radius*math.cos(end*math.pi*2)} {cam_high_radius*math.sin(end*math.pi*2)} "
        polar_data += f"L {polar_pos_x+cam_low_radius*math.cos((end+fall_time)*math.pi*2)} {cam_low_radius*math.sin((end+fall_time)*math.pi*2)} "
        last_pos = end
        polygon.append((start*x_scale, low+y))
        polygon.append(((start+rise_time)*x_scale, high+y))
        polygon.append((end*x_scale, high+y))
        polygon.append(((end+fall_time)*x_scale, low+y))

    start = polygon[0]
    path_data = f"M {start[0]} {start[1]} "
    for p in polygon[1:]:
        path_data += f"L {p[0]} {p[1]} "
    path_data += f"L {x_scale} {low+y} "
    print(f'<path d="{path_data}" fill="none" stroke="black"/>')

    print(f'<text x="{start[0]+x_scale}" y="{start[1]}" class="small">{c.signal_name}</text>')

    while 1.0-last_pos > 0.1:
        last_pos += 0.1
        polar_data += f"A {cam_low_radius} {cam_low_radius} 0 0 1 {polar_pos_x+cam_low_radius*math.cos(last_pos*math.pi*2)} {cam_low_radius*math.sin(last_pos*math.pi*2)} "

    polar_data += f"A {cam_low_radius} {cam_low_radius} 0 {1 if last_pos<0.5 else 0} 1 {polar_polygon[0][0]} {polar_polygon[0][1]} z"

    print(f'<path d="{polar_data}" fill="none" stroke="black"/>')
    polar_pos_x += cam_high_radius*2+10

    y += y_spacing
    camno += 1
    points = make_openscad_cam(combined_steps)
    point_array = ", ".join(f"[{p[0]:.3f}, {p[1]:.3f}]" for p in points)
    segment_pos_x = camno * 60;
    f.write(f"module cam_{camno}() {{\n")
    f.write(f"  difference() {{\n")
    f.write(f"    polygon(points=[{point_array}]);\n")
    f.write(f"    circle(d=bolt_circle_diameter-10);\n")
    f.write(f"    for(c=[0:7]) {{\n")
    f.write(f"       translate([bolt_circle_radius*cos(c*360*(1/8)+360*(1/16)), bolt_circle_radius*sin(c*360*(1/8)+360*(1/16))]) circle(d=8);\n")
    f.write(f"       rotate(360/16 + c*360/8) translate([0,-4]) square([bolt_circle_radius, 8]);\n")
    f.write(f"    }}\n")
    f.write(f"  }}\n")
    f.write(f"}}\n")

    cam_instances += (f"translate([{segment_pos_x}, 0]) rotate(-45) intersection() {{\n")
    cam_instances += (f"  cam_{camno}();\n")
    cam_instances += (f"  square([100,100]);\n")
    cam_instances += (f"}}\n")
    cam_instances += (f"translate([{segment_pos_x}-30, 0]) rotate(-90-45) intersection() {{\n")
    cam_instances += (f"  cam_{camno}();\n")
    cam_instances += (f"  translate([-100,0]) square([100,100]);\n")
    cam_instances += (f"}}\n")
    cam_instances += (f"translate([{segment_pos_x-15}-30, 100]) rotate(-180-45) intersection() {{\n")
    cam_instances += (f"  cam_{camno}();\n")
    cam_instances += (f"  translate([-100,-100]) square([100,100]);\n")
    cam_instances += (f"}}\n")
    cam_instances += (f"translate([{segment_pos_x-15}, 100]) rotate(-270-45) intersection() {{\n")
    cam_instances += (f"  cam_{camno}();\n")
    cam_instances += (f"  translate([0,-100]) square([100,100]);\n")
    cam_instances += (f"}}\n")

print('</svg>')
f.write(f"offset(r=kerf) {{\n")
f.write(cam_instances)
f.write(f"}}")

f.close()
