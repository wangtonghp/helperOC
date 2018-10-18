function uOpt = optCtrl(obj, ~, ~, deriv, uMode)
% uOpt = optCtrl(obj, t, y, deriv, uMode)

%% Input processing
if nargin < 5
  uMode = 'min';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

%% Optimal control
if strcmp(uMode, 'max')
  uOpt = (deriv{obj.dims==2}>=0)*obj.u_max + (deriv{obj.dims==2}<0)*(-obj.u_max);
elseif strcmp(uMode, 'min')
  uOpt = (deriv{obj.dims==2}>=0)*(-obj.u_max) + (deriv{obj.dims==2}<0)*obj.u_max;
else
  error('Unknown uMode!')
end

end