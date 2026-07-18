function varargout = computeRefDynFcn(varargin) %#codegen
%% PROTOTYPE
% varargout = computeRefDynFcn(varargin) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Backward-compatible lowercase wrapper. New callers should use ComputeRefDynFcn.
% -------------------------------------------------------------------------------------------------------------

[varargout{1:nargout}] = ComputeRefDynFcn(varargin{:});

end
