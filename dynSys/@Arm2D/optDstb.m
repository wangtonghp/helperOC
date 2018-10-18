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
  dOpt = (deriv{obj.dims==2}>=0)*obj.d_max + (deriv{obj.dims==2}<0)*(-obj.d_max);

elseif strcmp(dMode, 'min')
  dOpt = (deriv{obj.dims==2}>=0)*(-obj.d_max) + (deriv{obj.dims==2}<0)*obj.d_max;
else
  error('Unknown dMode!')
end

end