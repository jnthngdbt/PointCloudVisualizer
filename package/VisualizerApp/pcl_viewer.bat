
    REM The viewer window provides interactive commands; for help, press 'h' or 'H' from within the window.
    REM Syntax is: C:\Program Files\PCL 1.6.0\bin\pcd_viewer_release.exe <file_name 1..N>.<pcd or vtk> <options>
    REM   where options are:
    REM                      -bc r,g,b                = background color
    REM                      -fc r,g,b                = foreground color
    REM                      -ps X                    = point size (1..64)
    REM                      -opaque X                = rendered point cloud opacity (0..1)
    REM                      -ax n                    = enable on-screen display of XYZ axes and scale them to n
    REM                      -ax_pos X,Y,Z            = if axes are enabled, set their X,Y,Z position in space (default 0,0,0)
    REM                      -cam (*)                 = use given camera settings as initial view
    REM  (*) [Clipping Range / Focal Point / Position / ViewUp / Distance / Field of View Y / Window Size / Window Pos] or use a <filename.cam> that contains the same information.
    REM                      -multiview 0/1           = enable/disable auto-multi viewport rendering (default disabled)
    REM                      -normals 0/X             = disable/enable the display of every Xth point's surface normal as lines (default disabled)
    REM                      -normals_scale X         = resize the normal unit vector size to X (default 0.02)
    REM                      -pc 0/X                  = disable/enable the display of every Xth point's principal curvatures as lines (default disabled)
    REM                      -pc_scale X              = resize the principal curvatures vectors size to X (default 0.02)
    REM (Note: for multiple .pcd files, provide multiple -{fc,ps,opaque} parameters; they will be automatically assigned to the right file)
    
    pcl_viewer_release.exe %*