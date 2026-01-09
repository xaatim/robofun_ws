
from ast import Str
from utils.common import resolve_project_path

hello = resolve_project_path('src/robot_vision/utils/face_embeddings_tester.npy')
print(str(hello))