function [gfx] = plot_frame(origin, frame, style, sub)
% PLOT_FRAME - plots a co-ordinate frame with ref. to std. frame
% origin - [3x1] is the origin of the frame
% frame - [3x3] is the coordinate frame unit-basis matrix
% style - is the style code for the frame
% sub - is the subscript of the label (usually, frame name)

    hold on;
    
    gfx(1) = quiver3(origin(1), origin(2), origin(3), frame(1,1), frame(2,1), frame(3,1), style); % Plot x-axis
    gfx(2) = text(origin(1)+frame(1,1), origin(2)+frame(2,1), origin(3)+frame(3,1), 'x'); % X label
    
    gfx(3) = quiver3(origin(1), origin(2), origin(3), frame(1,2), frame(2,2), frame(3,2), style); % Plot y-axis
    gfx(4) = text(origin(1)+frame(1,2), origin(2)+frame(2,2), origin(3)+frame(3,2), 'y'); % Y Label
    
    gfx(5) = quiver3(origin(1), origin(2), origin(3), frame(1,3), frame(2,3), frame(3,3), style); % Plot z-axis
    gfx(6) = text(origin(1)+frame(1,3), origin(2)+frame(2,3), origin(3)+frame(3,3), 'z'); % Z Label
    
    gfx(7) = text(origin(1), origin(2), origin(3), ['C_{' sub '}']); % Frame Label
    
end