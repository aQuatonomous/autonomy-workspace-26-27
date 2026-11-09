"""Gazebo Label IDs for the course elements, used by the autolabel bounding-box cameras.

generate_course.py puts a gz-sim Label plugin on each element it includes, make_course_models.py
labels the vessel sign symbols and the banner numerals, and the answer key records this table so the recorder
(sim_autolabel) can turn box labels back into names. sim_autolabel/classes.yaml maps these names
to the CV team's classes; change the mapping there, not here.

IDs are 1-254: the bounding-box camera treats unlabelled visuals as background (255), and 0 is
kept free. Add new names at the end so existing recordings keep their meaning.
"""

LABELS = {
    'a0_red': 1,
    'a0_green': 2,
    'a0_black': 3,
    'a0_yellow': 4,
    'a2_red': 5,
    'a2_green': 6,
    'a2_black': 7,
    'a2_yellow': 8,
    'sur_mark_red': 9,
    'sur_mark_green': 10,
    'indicator_buoy_red': 11,
    'indicator_buoy_green': 12,
    'dock_indicator_red': 13,
    'dock_indicator_green': 14,
    'banner_1': 15,                 # the numeral on number banner 1, not the banner
    'banner_2': 16,
    'banner_3': 17,
    'vessel_sign_triangle': 18,     # black triangle on the yellow stationary vessel
    'vessel_sign_cross': 19,        # black plus on the black stationary vessel
    'dock': 20,
    # RoboBoat 2027
    'light_buoy_red': 21,           # light beacon buoy, lit red / green / blue, or off (black)
    'light_buoy_green': 22,
    'light_buoy_blue': 23,
    'light_buoy_black': 24,
    'a2_blue': 25,
    'a2_white': 26,
    'vessel_sign_circle': 27,
    'vessel_sign_square': 28,
}


def element_label(kind, color=None):
    """Label name for a course element, or None for elements that carry no label of their own.

    Stationary vessels and number banners return None: only their sign symbol or numeral is
    labelled, inside the model (the CV models box those, not the whole element).
    """
    if kind in ('vessel', 'banner'):
        return None
    name = 'dock' if kind == 'dock' else f'{kind}_{color}'
    return name if name in LABELS else None


def label_plugin(name):
    return (f'<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">'
            f'<label>{LABELS[name]}</label></plugin>')
