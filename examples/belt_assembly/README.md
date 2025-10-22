## Simulate round belt with Mujoco

```sh
bazel run @mujoco//:simulate -- $(bazel cquery --output=files //examples/belt_assembly:mujoco_xml/pulley.xml | xargs -I{} echo "$(bazel info execution_root)/{}")
```