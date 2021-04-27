cd ../../
mkdir compiledshaders
cd compiledshaders
mkdir dx11feature11
cd dx11feature11
mkdir fragment
cd ..
mkdir dx11feature9
cd dx11feature9
mkdir fragment
cd ../../shaders/fragment

CALL :SHADER fancy_cloth_diffuse
CALL :SHADER material_diffuse
CALL :SHADER outlinetext
CALL :SHADER particle_fog
CALL :SHADER pointsprite_diffuse
CALL :SHADER pointsprite_diffuse2
CALL :SHADER pointsprite_normal
CALL :SHADER sample_diffuse_and_texture
CALL :SHADER sample_diffuse_no_texture
CALL :SHADER screenquad
CALL :SHADER simple
CALL :SHADER simple_alpha
CALL :SHADER simple_color
CALL :SHADER simple_diffuse
CALL :SHADER simple_diffuse_multitex_heightmap
CALL :SHADER simple_diffuse_normal
CALL :SHADER simple_diffuse_tiled
CALL :SHADER simple_diffuse_uniform_color
CALL :SHADER simple_test
CALL :SHADER simple_turbulence
CALL :SHADER simple_uniform_color
CALL :SHADER simple_unlit_normals
CALL :SHADER simple_unlit_tangents
CALL :SHADER text
exit /b

:SHADER
CALL :COMPILE %1 UNLIT
CALL :COMPILE %1 DIRECTIONAL_LIGHT
CALL :COMPILE %1 AMBIENT_LIGHT
CALL :COMPILE %1 POINT_LIGHT
CALL :COMPILE %1 SPOT_LIGHT_NO_SHADOW
CALL :COMPILE %1 SPOT_LIGHT
CALL :COMPILE %1 NORMALS
CALL :COMPILE %1 DEPTH
exit /b

:COMPILE
set fragmentdefines9=/DRENDERER_FRAGMENT=1 /DRENDERER_D3D11=1 /DNO_SUPPORT_DDX_DDY=1 /DENABLE_VFACE=0 /DENABLE_VFACE_SCALE=0 /DRENDERER_WIN8ARM=1 /DPX_WINDOWS=1 /DENABLE_SHADOWS=0
set fragmentdefines11=/DRENDERER_FRAGMENT=1 /DRENDERER_D3D11=1 /DENABLE_VFACE=1 /DENABLE_VFACE_SCALE=1 /DPX_WINDOWS=1 /DENABLE_SHADOWS=0

"fxc.exe" /E"fmain" /Zpr /Gec /I"../include" %fragmentdefines9% /DPASS_%2 /Fo"../../compiledshaders/dx11feature9/fragment/%1.cg.PASS_%2.cso" /T ps_4_0_level_9_1 /nologo %1.cg
"fxc.exe" /E"fmain" /Zpr /Gec /I"../include" %fragmentdefines11% /DPASS_%2 /Fo"../../compiledshaders/dx11feature11/fragment/%1.cg.PASS_%2.cso" /T ps_4_0 /nologo %1.cg
