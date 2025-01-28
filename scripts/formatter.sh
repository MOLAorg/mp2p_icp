# formatter.sh
find \
    apps \
    mp2p_icp \
    mp2p_icp_map \
    mp2p_icp_filters \
    tests \
    -iname *.h -o -iname *.hpp -o -iname *.cpp -o -iname *.c \
    | xargs clang-format-14 -i
