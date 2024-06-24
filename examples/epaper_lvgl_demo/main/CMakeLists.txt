#set(imgpath ../lvui_common/img8b)
#set(fontpath ../lvui_common/fonts)
#set(lvpath ../lvui_common/)

#FILE(GLOB SRCS ${lvpath}/components/ui_*.c ${lvpath}/ui_*.c)
#FILE(GLOB FONTS ${fontpath}/ui_font*.c)
#FILE(GLOB IMGS ${imgpath}/ui_img*.c)

#set(src_dir ../lvui_ssd1680)
#FILE(GLOB SOURCES ${src_dir}/*.c ${src_dir}/components/*.c ${src_dir}/screens/*.c)
idf_component_register(SRCS "main.c" "lvgl_demo_ui.c" "img_bitmap.c" ${IMGS} ${FONTS} ${SOURCES} ${SRCS}
                       INCLUDE_DIRS "." ${src_dir}
                       REQUIRES driver)
