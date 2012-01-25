function world_map = generate_msrg_map()
    r_room = generate_r_room();
    l_room = generate_l_room();
    
    world_map = zeros(80, 25, 'uint8');
    h = 80;
    w = 25;
    
    world_map(:,:) = 255; % all to whitespace
    
    world_map(1, :) = 0; % top border
    world_map(h, :) = 0; %bottom border
    world_map(:, 1) = 0; %left border
    world_map(:, w) = 0; %right border
    
    for i = 1:6
        x = i*10+1;
        world_map(x:x+9, 1:10) = l_room;
        world_map(x:x+9, 16:25) = r_room;
    end
    
    world_map(71:80, 1:10) = flipud(l_room);
    world_map(71:80, 16:25) = flipud(r_room);
    
    world_map(71, 1:10) = 0;
    world_map(71, 16:25) = 0;
end

