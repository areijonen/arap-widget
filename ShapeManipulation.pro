# If this file exists, we are building inside the SDK source tree
exists($$PWD/../../cornerstone.pri):include($$PWD/../../cornerstone.pri)
else {
  exists($$(CORNERSTONE_SDK_ROOT)/cornerstone.pri):include($$(CORNERSTONE_SDK_ROOT)/cornerstone.pri)
  else {
    exists($$(__CORNERSTONE_ROOT__)/cornerstone.pri):include($$(__CORNERSTONE_ROOT__)/cornerstone.pri)
    else:include(/opt/cornerstone-__CORNERSTONE_SHORT_VERSION__/cornerstone.pri)
  }
}

SOURCES += Main.cpp
SOURCES += ShapeManipulationWidget.cpp

HEADERS += ShapeManipulationWidget.hpp


LIBS += $$LIB_PATTERNS
LIBS += $$LIB_NIMBLE
LIBS += $$LIB_RADIANT
LIBS += $$LIB_VALUABLE
LIBS += $$LIB_LUMINOUS
LIBS += $$LIB_RESONANT
LIBS += $$LIB_VIDEODISPLAY
LIBS += $$LIB_MULTITOUCH
LIBS += $$LIB_STYLISH
LIBS += $$LIB_MULTIWIDGETS
LIBS += $$LIB_WEB_BROWSER
LIBS += $$LIB_WEB_BROWSER_CEF
LIBS += $$LIB_OPENGL
LIBS += $$LIB_MULTISTATEDISPLAY
LIBS += $$LIB_SCRIPT
LIBS += $$LIB_MUSHY

win32 {
  LIBS += -llibsndfile-1
  CONFIG += console
}

unix {
  CONFIG += link_pkgconfig
  PKGCONFIG += sndfile
}

