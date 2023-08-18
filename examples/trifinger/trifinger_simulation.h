#pragma once



namespace dairlib {
namespace examples {
namespace trifinger {

template <typename T>
class TrifingerPlant : public drake::systems::LeafSystem<T> {
 public:
  TrifingerPlant();

 private:
  int a;
};

}  // namespace trifinger
}  // namespace examples
}  // namespace dairlib