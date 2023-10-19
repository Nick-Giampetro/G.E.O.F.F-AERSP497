@echo "  we are in a repo"
@git diff-index --quiet HEAD
@IF %ERRORLEVEL% NEQ 0 (
	@echo   repo is dirty
	@SET tag=m
)
@FOR /F "tokens=* USEBACKQ" %%F IN (`git show -s --format^=%%ci`) DO ( SET date=%%F)
@FOR /F "tokens=* USEBACKQ" %%F IN (`git show -s --format^=%%h`) DO ( SET hash=%%F )
@SET verstr=%tag%%hash% %date%
@echo "  Version string: %verstr%"
@echo #define WORK_VERSION "%verstr%" > ./workversion.h
