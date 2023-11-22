elementNames = out.logsout.getElementNames;

i = 2;

timeStart = 0;
timeEnd   = 250;
for i=1:length(elementNames)
    elementName = elementNames{i};
    selectedTimeSeries  = out.logsout{i}.Values;
    deltaTime = out.logsout{i}.Values.TimeInfo.Increment;
    trimmedTimeSeries   = getsampleusingtime(selectedTimeSeries ,timeStart,timeEnd);
    out.logsout{i}.Values = trimmedTimeSeries;
end