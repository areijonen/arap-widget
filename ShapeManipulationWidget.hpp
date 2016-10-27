#ifndef SHAPEMANIPULATIONWIDGET_WIDGET_HPP
#define SHAPEMANIPULATIONWIDGET_WIDGET_HPP

#include <MultiWidgets/ImageWidget.hpp>

class ShapeManipulationWidget : public MultiWidgets::Widget
{
public:
  ShapeManipulationWidget();

  virtual void renderContent(Luminous::RenderContext & r) const override;
  virtual bool isInside(Nimble::Vector2 v) const override;
  virtual Nimble::Rect boundingRect() const override;

  virtual void update(const MultiWidgets::FrameInfo &frameInfo) override;

  virtual void processFingers(MultiWidgets::GrabManager & gm, const MultiWidgets::FingerArray & fingers, float dt) override;
  virtual void interactionEnd(MultiWidgets::GrabManager &gm) override;
private:
  class D;
  std::unique_ptr<D> m_d;
};


#endif
