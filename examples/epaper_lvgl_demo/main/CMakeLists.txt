# set(lvui_path ../../../../../main/lvui)
# set(imgpath ${lvui_path}/img8b)
# set(fontpath ${lvui_path}/fonts)
# set(lvpath ${lvui_path})

# FILE(GLOB SRCS ${lvpath}/screens/ui_*.c ${lvpath}/ui_*.c)
# FILE(GLOB FONTS ${fontpath}/ui_font*.c)
# FILE(GLOB IMGS ${imgpath}/ui_img*.c)

FILE(GLOB SRCS *.c)

idf_component_register(SRCS ${SRCS}
                       INCLUDE_DIRS "." ${src_dir}
                       REQUIRES driver)
