scoms = instrfindall;
if isempty(scoms) == 0
stopasync(scoms);
fclose(scoms);
delete(scoms);

end