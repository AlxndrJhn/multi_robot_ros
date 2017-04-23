import os
from xml.dom import minidom
# Do the map  in http://editor.method.ac/
# by specifying the document size, and insering rectangles.
from config.maps import points_to_rectangle


# Read the document

def svg_to_map(file_name):
    """
    Do the map  in http://editor.method.ac/
    by specifying the document size, and insering rectangles.
    Two ellipses for the start and goal points.
    :param file_name:
    :return:
    """
    if file_name.startswith('/'):
        file_name = file_name
    else:
        file_name = os.path.join(os.path.dirname(__file__), '../maps/%s' % file_name)
    doc = minidom.parse(file_name)  # parseString also exists


    # Environment
    svg = doc.getElementsByTagName('svg')[0]
    w = float(svg.getAttribute('width'))
    h = float(svg.getAttribute('height'))
    E = points_to_rectangle((0, 0), (w, h))



    ## Obstacles
    svg_rectangles = doc.getElementsByTagName('rect')

    Obs = []
    Obs_type = []
    for rec in svg_rectangles:
        if rec.getAttribute('id') == 'canvas_background':
            continue

        x, y = float(rec.getAttribute('x')), h - float(rec.getAttribute('y'))
        rw, rh = float(rec.getAttribute('width')), float(rec.getAttribute('height'))
        # print (x, y, rw, rh)
        obstacle = points_to_rectangle((x, y), (x + rw, y - rh))
        color = rec.getAttribute('fill')
        if color=='':
            color = rec.getAttribute('style').split(';')[0].split(':')[1]
        if color==u'#000000':
            obstacle_type = 0 # black, static, known
        elif color==u'#0000ff':
            obstacle_type = 1 # blue: static, discoverable (unknown at beginning)
        else:
            obstacle_type = 0  # polygon octacles
        Obs.append(obstacle)
        Obs_type.append(obstacle_type)

    # Start and goal
    ellipses = doc.getElementsByTagName('ellipse')
    e_start, e_goal = ellipses[0], ellipses[1]
    start = float(e_start.getAttribute('cx')), h - float(e_start.getAttribute('cy'))
    goal = float(e_goal.getAttribute('cx')), h - float(e_goal.getAttribute('cy'))

    doc.unlink()

    if any(Obs_type):
        return E, Obs, Obs_type, start, goal
    else:
        return E, Obs, start, goal
