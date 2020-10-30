function code = LatexMatrix(X)
    code = "\begin{bmatrix}" + newline;
    rows = size(X, 1);
    cols = size(X, 2);
    for i=1:rows
        for j=1:cols           
            if j == cols
                code = code + X(i, j) + " \\";
            else
                code = code + X(i, j) + " & ";
            end
        end
        code = code + newline;
    end
    code = code + "\end{bmatrix}" + newline;
    fprintf(code);
end