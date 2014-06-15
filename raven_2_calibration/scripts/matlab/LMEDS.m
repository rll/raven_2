%LMEDS - least median of squares robust estimation algorithm
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%
%    [Model] = LMEDS( mData, ModelFunc, nSampLen, ResidFunc, nIter )
%    ---------------------------------------------------------------------------------
%    Arguments:
%           mData - matrix of data, where each column-vector is point
%           ModelFunc - handle to Model Creating function. It must create a
%                   model from nSampLen column-vectors organized in
%                   matrix
%           nSampLen - number of point for ModelFunc
%           ResidFunc - handle to Residuum calculating function. As
%                   argument this function takes model, calculated by
%                   ModelFunc, and matrix of data (all or maybe part of it)
%           nIter - number of iterations for MSAC algorithm
%    Return:
%           Model - approximate model for this data


function [Model] = LMEDS( mData, ModelFunc, nSampLen, ResidFunc, nIter )


nResIter = nIter;
    
% Cheking arguments
if length(size(mData)) ~=2
    error('Data must be organized in column-vecotors massive');
end

nDataLen = size(mData, 2);

if( nDataLen < nSampLen )
    error('Not enough data to compute model function');
end

% Initialization
Model = NaN;
vMask = zeros([1 nDataLen]);

dMinPenalty = Inf;


% Main cycle
for i = 1:nIter
    % 1. Sampling
    Sample = randsample(1:1:nDataLen,nSampLen);
    
    % 2. Creating model
    ModelSet = feval(ModelFunc, mData(:, Sample));

    for iModel = 1:size(ModelSet, 3)
        
        CurModel = ModelSet(:, :, iModel);
    
        % 3. Model estimation    
        CurResid    = abs(feval(ResidFunc, CurModel, mData));
        dCurPenalty = median(CurResid);
    
        % 4. The best is selected
        if dMinPenalty > dCurPenalty

            % Save some parameters
            dMinPenalty = dCurPenalty;
            Model = CurModel;
        end
    end
end

return; 
