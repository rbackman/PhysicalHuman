// generated by Fast Light User Interface Designer (fluid) version 2.1000

#include "game_fluid.h"

inline void GameWindowFluid::cb_ui_jump_distance_i(fltk::ValueSlider*, void*) {
  event(evSceneAdjust);
}
void GameWindowFluid::cb_ui_jump_distance(fltk::ValueSlider* o, void* v) {
  ((GameWindowFluid*)(o->parent()->parent()->user_data()))->cb_ui_jump_distance_i(o,v);
}

inline void GameWindowFluid::cb_ui_jump_height_i(fltk::ValueSlider*, void*) {
  event(evSceneAdjust);
}
void GameWindowFluid::cb_ui_jump_height(fltk::ValueSlider* o, void* v) {
  ((GameWindowFluid*)(o->parent()->parent()->user_data()))->cb_ui_jump_height_i(o,v);
}

inline void GameWindowFluid::cb_Jump_i(fltk::Button*, void*) {
  event(evPlay);
}
void GameWindowFluid::cb_Jump(fltk::Button* o, void* v) {
  ((GameWindowFluid*)(o->parent()->parent()->user_data()))->cb_Jump_i(o,v);
}

inline void GameWindowFluid::cb_Reset_i(fltk::Button*, void*) {
  event(evReset);
}
void GameWindowFluid::cb_Reset(fltk::Button* o, void* v) {
  ((GameWindowFluid*)(o->parent()->parent()->user_data()))->cb_Reset_i(o,v);
}

inline void GameWindowFluid::cb_Quit_i(fltk::Button*, void*) {
  event ( evQuit );
}
void GameWindowFluid::cb_Quit(fltk::Button* o, void* v) {
  ((GameWindowFluid*)(o->parent()->parent()->user_data()))->cb_Quit_i(o,v);
}

GameWindowFluid::GameWindowFluid() {
  fltk::Window* w;
   {fltk::Window* o = ui_window = new fltk::Window(668, 37, "Physical Human");
    w = o;
    o->set_vertical();
    o->shortcut(0xff1b);
    o->user_data((void*)(this));
    o->begin();
     {fltk::Group* o = new fltk::Group(2, 0, 666, 37, "Viewer");
      o->box(fltk::DOWN_BOX);
      o->begin();
       {fltk::ValueSlider* o = ui_jump_distance = new fltk::ValueSlider(283, 5, 165, 30, "distance");
        o->minimum(0.5);
        o->maximum(2);
        o->value(1);
        o->callback((fltk::Callback*)cb_ui_jump_distance);
        o->align(fltk::ALIGN_LEFT);
      }
       {fltk::ValueSlider* o = ui_jump_height = new fltk::ValueSlider(498, 5, 166, 30, "height");
        o->minimum(-1);
        o->callback((fltk::Callback*)cb_ui_jump_height);
        o->align(fltk::ALIGN_LEFT);
      }
       {fltk::Button* o = new fltk::Button(143, 2, 75, 35, "Jump");
        o->callback((fltk::Callback*)cb_Jump);
      }
       {fltk::Button* o = new fltk::Button(73, 2, 65, 35, "Reset");
        o->callback((fltk::Callback*)cb_Reset);
      }
       {fltk::Button* o = new fltk::Button(3, 2, 65, 35, "Quit");
        o->callback((fltk::Callback*)cb_Quit);
      }
      o->end();
    }
    o->end();
    o->resizable(o);
  }
}