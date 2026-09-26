import sys
from pathlib import Path

# allow `from test_gestures import body` and `import passer` without installing
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parents[1]))
