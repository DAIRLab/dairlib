#pragma once

namespace dairlib {
namespace systems {

class ACOM {
 public:
  ACOM();

  drake::VectorX<double> EvalQBaseAcom() const;
  template <typename T>
  drake::MatrixX<T> EvalJQBaseAcomEwrtAcom() const;
  template <typename T>
  drake::MatrixX<T> EvalJOmegaBaseAcomEwrtAcom() const;
  template <typename T>
  drake::MatrixX<T> EvalJOmegaWorldAcomEwrtWorld() const;
  template <typename T>
  drake::VectorX<T> EvalJdotVOmegaWorldAcomEwrtWorld() const;

 private:
  template <typename T>
  virtual void getJQx(const drake::VectorX<T>&, drake::MatrixX<T>&) = 0;
  template <typename T>
  virtual void getJQy(const drake::VectorX<T>&, drake::MatrixX<T>&) = 0;
  template <typename T>
  virtual void getJQz(const drake::VectorX<T>&, drake::MatrixX<T>&) = 0;
};

}  // namespace systems
}  // namespace dairlib