from shapely.geometry import Polygon


def map_empty(w=200, h=140):
    E = Polygon([(0, 0), (w, 0), (w, h), (0, h)])

    # Obstacles
    Obs = []

    # start
    start = (.1 * w, .1 * h)
    goal = (.9 * w, .1 * h)

    return E, Obs, start, goal


def map1(w=200, h=140):
    E = Polygon([(0, 0), (w, 0), (w, h), (0, h)])

    # Obstacles
    obs1 = Polygon([(.15 * w, 0), (.4 * w, 0), (.4 * w, .3 * h), (.3 * w, .3 * h)])
    obs2 = Polygon([(.2 * w, h), (.5 * w, h), (.5 * w, .8 * h), (.4 * w, .8 * h)])
    obs3 = Polygon([(.0 * w, .4 * h), (.2 * w, .4 * h), (.2 * w, .60 * h), (.0 * w, .60 * h)])
    obs4 = Polygon([(.6 * w, 0), (.7 * w, 0), (.7 * w, .3 * h), (.6 * w, .3 * h)])
    obs5 = Polygon([(.8 * w, .5 * h), (1 * w, .5 * h), (1 * w, .60 * h), (.8 * w, .60 * h)])
    Obs = [obs1, obs2, obs3, obs4, obs5]

    # start
    start = (.1 * w, .1 * h)
    goal = (.9 * w, .1 * h)

    return E, Obs, start, goal


def points_to_rectangle(p1, p2):
    return Polygon([p1, (p2[0], p1[1]), p2, (p1[0], p2[1])])


def map_simple_lab(w=200, h=140):
    """
    Simple labyrinth.
    :param w:
    :param h:
    :return:
    """
    E = Polygon([(0, 0), (w, 0), (w, h), (0, h)])

    # Obstacles
    Obs = [points_to_rectangle((40, 0), (50, 60)),
           points_to_rectangle((80, 40), (90, h)),
           points_to_rectangle((120, 0), (140, h / 2)),
           points_to_rectangle((0, 100), (70, h)),
           # obs_restangle((40, 0), (50, 30)),
           # obs_restangle((40, 0), (50, 30))
           ]

    # start
    start = (.1 * w, .1 * h)
    goal = (.9 * w, .1 * h)

    return E, Obs, start, goal
