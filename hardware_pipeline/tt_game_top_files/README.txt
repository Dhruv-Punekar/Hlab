Table-tennis demo (tt_top) - files
==================================
Top module : tt_top                    (this folder)
Constraints: constraints.xdc           (this folder)

RTL, this folder (demo-specific):
  tt_top.v            board top: MMCM (100 MHz render, 25.175 MHz pixel) + tt_game
  tt_game.v           pipeline instance, game, animator, overlay, clock crossings
  tt_game_core.v      game logic (serve, rally, hit window, reach, score, tilt aim)
  tt_obj_animator.v   rewrites racket / ball vertices before each render
  tt_view_loader.v    writes the selected camera view (static scene + matrix)
  tt_score_overlay.v  score panel on the VGA output, frame ticks
  tt_buttons.v        button sync + debounce + press events
  tt_sound.v          hit / bounce / point sounds on the audio jack
  tt_tilt.v           accelerometer -> P1 aim position
RTL, shared:
  hardware_pipeline/hardware_pipeline.v and its modules.
  hardware_pipeline/game_files/adxl362_reader.v (accelerometer SPI reader)
  tt_top uses the single-buffer mode (DOUBLE_BUF = 0) with the options
  CLEAR_COLOR_TOP / CLEAR_COLOR_BOTTOM / CLEAR_SPLIT_Y (sky / floor clear)
  and VTX_WRITE_EN = 1 (moving objects). All of these default to off, so the
  other demos are unaffected.

Data (mem_files/tt_game/, add to the Vivado project):
  vertices.mem, matrix.mem, tt_views.mem, tt_scene_map.vh, tt_game_map.vh
  and mem_files/recip_lut_q1_17.mem
  vertex_memory / matrix_memory load the fixed names "vertices.mem" and
  "matrix.mem", so build this demo in its own Vivado project.

Controls
  P1 (near, red): BTNR forehand, BTNL backhand.  P2 (far, black): BTNU, BTND.
  BTNC short press = next camera view (player 1 / side / player 2),
  BTNC held 1 s = new game.  First to 11 points, serve changes every 2.
  SW0 P2 by computer, SW1 P1 by computer, SW2 P1 aims by tilting the board,
  SW3 mute, SW4 invert tilt, SW5 use the accelerometer Y axis.

Testbenches (simulation only): tb_tt_scene.v, tb_tt_game_core.v, tb_tt_game.v
Generators: mem_files/tt_game/gen_tt_scene.py, tt_game_model.py
Design notes: design_doc/design_doc_tt_scene.txt, design_doc_tt_game.txt
