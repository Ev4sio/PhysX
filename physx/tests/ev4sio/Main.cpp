#if defined(_WIN32)
extern "C" __declspec(dllimport) int ev4sio_run_smoke_tests();
#else
extern "C" int ev4sio_run_smoke_tests();
#endif

int main()
{
    return ev4sio_run_smoke_tests();
}
