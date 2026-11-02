Table-tennis demo (tt_top) - files
==================================
Top module : tt_top                    (this folder)
Constraints: constraints.xdc           (this folder)

RTL, this folder (demo-specific):
  tt_top.v            board top: MMCM (100 MHz render, 25.175 MHz pixel) + tt_game
  tt_game.v           pipeline instance, game, animator, overlay, clock crossings
  tt_game_core.v      game logic (serve, rally, hit window, score), pixel clock
  tt_obj_animator.v   rewrites racket / ball vertices before each render
  tt_score_overlay.v  score panel on the VGA output, frame ticks
  tt_buttons.v        button sync + debounce + press events
RTL, shared (hardware_pipeline/): hardware_pipeline.v and its modules.
  tt_top uses the single-buffer mode (DOUBLE_BUF = 0) with the options
  CLEAR_COLOR_TOP / CLEAR_COLOR_BOTTOM / CLEAR_SPLIT_Y (sky / floor clear)
  and VTX_WRITE_EN = 1 (moving objects). All of these default to off, so the
  other demos are unaffected.

Data (mem_files/tt_game/, add to the Vivado project):
  vertices.mem, matrix.mem, tt_scene_map.vh, tt_game_map.vh
  and mem_files/recip_lut_q1_17.mem
  vertex_memory / matrix_memory load the fixed names "vertices.mem" and
  "matrix.mem", so build this demo in its own Vivado project.

Testbenches (simulation only): tb_tt_scene.v, tb_tt_game_core.v, tb_tt_game.v
Generators: mem_files/tt_game/gen_tt_scene.py, tt_game_model.py
Design notes: design_doc/design_doc_tt_scene.txt, design_doc_tt_game.txt
