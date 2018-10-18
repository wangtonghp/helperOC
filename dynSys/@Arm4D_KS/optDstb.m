function dOpt = optDstb(obj, ~, ~, deriv, dMode)
% dOpt = optCtrl(obj, t, y, deriv, dMode)


%% Input processing
if nargin < 5
  dMode = 'max';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end


%% Optimal control
if strcmp(dMode, 'max')
  dOpt{1} = (deriv{obj.dims==3}>=0)*obj.dMax + (deriv{obj.dims==3}<0)*(-obj.dMax);
  dOpt{2} = (deriv{obj.dims==4}>=0)*obj.dMax + (deriv{obj.dims==4}<0)*(-obj.dMax);

elseif strcmp(dMode, 'min')
  dOpt{1} = (deriv{obj.dims==3}>=0)*(-obj.dMax) + (deriv{obj.dims==3}<0)*obj.dMax;
  dOpt{2} = (deriv{obj.dims==4}>=0)*(-obj.dMax) + (deriv{obj.dims==4}<0)*obj.dMax;
else
  error('Unknown dMode!')
end

end