% Little utility function to avoid plotting over figure windows you still
% have open. There may be a simpler way to do this but oh well.

function Nf = nextFig
figNums = get(allchild(0),'Number');
len = length(figNums);
if len == 0
    Nf = 1;
elseif len == 1
    Nf = figNums + 1;
else
    Nf = max(cell2mat(figNums)) + 1;
end