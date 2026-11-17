add_library(ps2kbd STATIC)

target_sources(ps2kbd PRIVATE
${CMAKE_CURRENT_LIST_DIR}/ps2kbd_mrmltr.cpp
${CMAKE_CURRENT_LIST_DIR}/ps2kbd_mrmltr.h
    ${CMAKE_CURRENT_LIST_DIR}/ps2kbd_wrapper.cpp
    ${CMAKE_CURRENT_LIST_DIR}/ps2kbd_wrapper.h
)

target_link_libraries(ps2kbd PRIVATE hardware_pio hardware_clocks)

# Board define and the clock pin, which picks the PIO program (data on the
# next pin): M1 clock 0, M2 and Z0 clock 2
target_compile_definitions(ps2kbd PRIVATE BOARD_${BOARD_VARIANT}
    KBD_CLOCK_PIN=${PS2_PIN_CLK} PS2_PIN_CLK=${PS2_PIN_CLK} PS2_PIN_DATA=${PS2_PIN_CLK}+1)

target_include_directories(ps2kbd PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
)

target_include_directories(ps2kbd PRIVATE
    ../../src
)

pico_generate_pio_header(ps2kbd 
${CMAKE_CURRENT_LIST_DIR}/ps2kbd_mrmltr.pio
)

pico_generate_pio_header(ps2kbd 
${CMAKE_CURRENT_LIST_DIR}/ps2kbd_mrmltr2.pio
)
