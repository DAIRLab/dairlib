import pyvista
import tetgen

name = 'round_belt'

filename = f'urdf/round_belt_task/{name}.obj'
reader = pyvista.get_reader(filename)
mesh = reader.read()
mesh.save(f'{name}.ply', binary=False)

tgen = tetgen.TetGen(f'{name}.ply')
# Same as this command-line option: tgen.tetrahedralize(switches='qYMkV')
tgen.tetrahedralize(quality=True, nobisect=True, nomergefacet=True,
                    nomergevertex=True, vtksurfview=True, vtkview=True,
                    verbose=True)

tgen.write(f'{name}.vtk', binary=False)
