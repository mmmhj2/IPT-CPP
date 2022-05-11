The client is used to perform calculations on a Windows host.
It is built with MSYS2-MINGW64.

To successfully build the project, follow these steps:
* Install OpenCV and nlohmann-json on msys2-mingw64.
* Manually install AprilTag on Windows
	- Clone the repo of AprilTag.
	- Change its Makefile to generate "libapriltag.dll.a" and "libapriltag.so".
	- Use `make install` to install AprilTag and copy the headers to the `include` directory of msys2.
		+ Alternatively, you can modify the `PREFIX` of Makefile.
		+ Do not try to generate all the demos, or the makefile won't recognize OpenCV libraries installed in MSYS2-MINGW64.
* Use cmake to generate caches and then compile.
	- Recommend to use `-G"MSYS Makefiles"`.
