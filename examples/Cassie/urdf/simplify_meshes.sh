cd meshes-orig
for i in ./agility/*.obj; do
    [ -f "$i" ] || break
    #echo ../meshes/$i
    meshlabserver -i $i -o ../meshes/$i -s ../mesh_reduction_filter.mlx
done

for i in ./*.obj; do
    [ -f "$i" ] || break
    #echo ../meshes/$i
    meshlabserver -i $i -o ../meshes/$i -s ../mesh_reduction_filter.mlx
done