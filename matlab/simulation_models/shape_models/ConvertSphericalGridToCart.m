function [dSphereGridCoords] = ConvertSphericalGridToCart(dAzValues, dElValues, dRangeValues)
arguments
    dAzValues
    dElValues
    dRangeValues
end

dSphereGridCoords = zeros(length(dAzValues), 3);

if isscalar(dRangeValues)
    [dSphereGridCoords(:,1), dSphereGridCoords(:,2), dSphereGridCoords(:,3)] = sph2cart(dAzValues, dElValues, dRangeValues * ones(length(dAzValues), 1));

elseif isvector(dRangeValues)
    [dSphereGridCoords(:,1), dSphereGridCoords(:,2), dSphereGridCoords(:,3)] = sph2cart(dAzValues, dElValues, dRangeValues);
end

end
