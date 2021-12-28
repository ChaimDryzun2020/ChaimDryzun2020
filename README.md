SkillReal's Cameras Code


Code Structure
        
        All the C++ source code is located in the Source directory. 
        Go to Source\CamerasTest\ and open the CamerasTest.sln. This is the main solution containg all the c++ projects.
        The CamerasTest project is an excutable which creates an exe file in the Bin directory. Here we can add all kind of unit tests.
       
        The basic code creates C++ Lib file which are saved in the Libs directory.
        All the h files are copied to the Include directory.
        
        There are dependencies on some more basic (common) SkillReal code which is not part of the Camera's projects.
        The lib files of this code is present in the Lib directory. 
        The h files of this code is present in the Include directory.
        If you need the source code of the basic (common) code - contact SkillReal.
        
        The API code and Camera DLL projects creates DLL which are saved in the API\DLL\ directory.
        In the API directory there is a C# dot net code which is a C# wrapper for the C++ DLLs.
        
        In the Tool directory there is a C++ MFC based tool running and testing the code. 
        It creates an exe file in the Bin directory.        


Dependencies:
        
        1. Eigen3 3.3.9 --> https://eigen.tuxfamily.org/index.php?title=Main_Page
        2. Boost 1.75.0 --> https://github.com/boostorg/boost
        3. FLANN 2019.04.07 --> https://github.com/flann-lib/flann
        4. JSONCpp 1.9.4 --> https://github.com/open-source-parsers/jsoncpp
        5. OpenGL 0.08 --> https://www.opengl.org/
        6. GLM 0.9.9.81 --> https://github.com/g-truc/glm
        7. GL3W 2018.05.31-2 --> https://github.com/skaslev/gl3w
        8. GLEW 2.1.0-10 --> http://glew.sourceforge.net/
        9. GLFW3 3.3.4 --> https://www.glfw.org/
        10. OpenCV [with Eigen, FFMPEG, JASPER, JPEG, LAPACK, OpneGL, QT5, TIFF and VTK dependencies] 4.5.2 --> https://opencv.org/
        11. PCL 1.11.14 [With QHullL, OpenGL, QT5, VTK and OpenNI2 dependencies] --> https://pointclouds.org
        
        As we wre working with Windows and with Visual Studio IDE - I reccoment working with 
        VCPKG --> https://github.com/microsoft/vcpkg (a cross-platform open source package manager by Microsoft).
        I am working with VCPKG as library managment tool and it is integrated with Visual Studio so
        I do not need to explicitly include and link these libraries. (So, If you are not working with VCPKG,
        pay attension that these libraries are not explicitly included and linked to the code and you will have to do it manually). 
        
        
        
        
    
        
