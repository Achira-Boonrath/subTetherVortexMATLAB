function h = draw_boxwing_satellite(xc, yc, theta_rot, params, varargin)

if nargin < 5 || isempty(varargin)
    h = [];
else
    h = varargin{1};
end

% DRAW_BOXWING_SATELLITE
% Draws or updates a box-wing satellite at a given pose.
%
% INPUTS:
%   xc, yc      - Position in LVLH frame
%   theta_rot   - Rotation angle [rad]
%   h           - Struct of patch handles (empty if first call)
%   params      - Struct with geometry:
%                   .BodySize
%                   .PanelLength
%                   .PanelWidth
%
% OUTPUT:
%   h           - Updated struct of patch handles

%% Unpack parameters
bs = params.BodySize;
pl = params.PanelLength;
pw = params.PanelWidth;

%% --- Geometry in body frame ---

% Main body
body = bs/2 * [-1 -1;
                1 -1;
                1  1;
               -1  1]';

% Panels
left_panel = [ -bs/2, -pw/2;
               -bs/2 - pl, -pw/2;
               -bs/2 - pl,  pw/2;
               -bs/2,  pw/2 ]';

right_panel = [ bs/2, -pw/2;
                bs/2 + pl, -pw/2;
                bs/2 + pl,  pw/2;
                bs/2,  pw/2 ]';

%% --- Rotation matrix ---
R = [cos(theta_rot) -sin(theta_rot);
     sin(theta_rot)  cos(theta_rot)];

%% --- Transform geometry ---
body_rot  = R * body        + [xc; yc];
left_rot  = R * left_panel  + [xc; yc];
right_rot = R * right_panel + [xc; yc];

%% --- Create or update patches ---
if isempty(h)
    % Create new patches
    h.body  = patch(body_rot(1,:),  body_rot(2,:),  'g');
    h.left  = patch(left_rot(1,:),  left_rot(2,:),  'b');
    h.right = patch(right_rot(1,:), right_rot(2,:), 'b');
else
    % Update existing patches
    set(h.body,  'XData', body_rot(1,:),  'YData', body_rot(2,:));
    set(h.left,  'XData', left_rot(1,:),  'YData', left_rot(2,:));
    set(h.right, 'XData', right_rot(1,:), 'YData', right_rot(2,:));
end

end