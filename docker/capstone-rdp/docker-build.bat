@echo off

setlocal
call "%~dp0..\..\scripts\abspath.bat" "%~dp0..\.."
call "%~dp0docker-id.bat"
docker build --build-arg base=%DockerUserName%/%DockerImageBaseName%:%DockerImageTag% --tag=%DockerUserName%/%DockerImageName%:%DockerImageTag% --file="%~dp0Dockerfile" "%AbsPath%"
endlocal