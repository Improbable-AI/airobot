import os

from airobot.utils.common import list_class_names

cur_path = os.path.dirname(os.path.abspath(__file__))
cls_name_to_path = list_class_names(cur_path)
