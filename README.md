# py_Pioneer-3DX :warning:

# WARNING: ongoing project!

To run locally:

In your projects directory, run the following command:

`> git clone https://github.com/NERo-AuRoRA/py_Pioneer-3DX.git`

Open the project, and in your terminal, install the requirements:

`> pip install requirements.txt`

Now, in yourn Python console, create a class object and use its method set_pose using the following commands:

`>>> from Pioneer3DX.Pioneer3DX import Pioneer3DX`

`>>> import matplotlib.pyplot as plt`

`>>> import numpy as np`

`>>> p = Pioneer3DX()`

`>>> axis = [-5, 5, -5, 5]`

`>>> fig = plt.figure(1, figsize=[5, 5])`

`>>> p.Draw(fig, axis)`

`>>> p.set_pose([x, y])`
