HeliMexHandle = [];
try
    helicopter('CloseAll');
    HeliMexHandle = helicopter('open', 'spixo1');               % open port
    helicopter('setlimbparams', HeliMexHandle, 'angleoffset', 0.0, 0.0);
    helicopter('write', HeliMexHandle, [0.0 0.0]);
catch err
    warning(err.message);
end;