from cams import cams

y = 0
low = 0
x_scale = 1000
y_spacing = 100
high = -50 # Negative is up in SVG
rise_time = 0.02
fall_time = 0.01
height = len(cams)*y_spacing+100

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

for c in cams:
    polygon = [(0,y)]
    combined_steps = combine(c.steps)
    for (start, length) in combined_steps:
        polygon.append((start*x_scale, low+y))
        polygon.append(((start+rise_time)*x_scale, high+y))
        polygon.append(((start+length)*x_scale, high+y))
        polygon.append(((start+length+fall_time)*x_scale, low+y))

    start = polygon[0]
    path_data = f"M {start[0]} {start[1]} "
    for p in polygon[1:]:
        path_data += f"L {p[0]} {p[1]} "
    path_data += f"L {x_scale} {low+y} "
    print(f'<path d="{path_data}" fill="none" stroke="black"/>')

    print(f'<text x="{start[0]+x_scale}" y="{start[1]}" class="small">{c.signal_name}</text>')
    y += y_spacing
print('</svg>')
