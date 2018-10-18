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
  uOpt{1} = (deriv{obj.dims==3}>=0)*obj.uMax + (deriv{obj.dims==3}<0)*(-obj.uMax);
  uOpt{2} = (deriv{obj.dims==4}>=0)*obj.uMax + (deriv{obj.dims==4}<0)*(-obj.uMax);
elseif strcmp(uMode, 'min')
  uOpt{1} = (deriv{obj.dims==3}>=0)*(-obj.uMax) + (deriv{obj.dims==3}<0)*obj.uMax;
  uOpt{2} = (deriv{obj.dims==4}>=0)*(-obj.uMax) + (deriv{obj.dims==4}<0)*obj.uMax;
else
  error('Unknown uMode!')
end

end