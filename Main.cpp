#include "ShapeManipulationWidget.hpp"

#include <MultiWidgets/Application.hpp>

int main(int argc, char ** argv)
{
  MultiWidgets::Application app;

  if(!app.init(argc, argv))
    return 1;

  auto w = MultiWidgets::create<ShapeManipulationWidget>();
  app.mainLayer()->addChild(w);

  return app.run();
}
