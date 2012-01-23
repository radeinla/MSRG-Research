function world_map = generate_msrg_map()
    r_room = generate_r_room();
    l_room = generate_l_room();
    
    world_map = zeros(800, 250, 'uint8');
    h = 800;
    w = 250;
    
    world_map(:,:) = 255; % all to whitespace
    
    world_map(1:5, :) = 0; % top border
    world_map(h-4:h, :) = 0; %bottom border
    world_map(:, 1:5) = 0; %left border
    world_map(:, w-4:w) = 0; %right border
    
    for i = 1:6
        x = i*100+1;
        world_map(x:x+99, 1:100) = l_room;
        world_map(x:x+99, 151:250) = r_room;
    end
    
    world_map(701:800, 1:100) = flipud(l_room);
    world_map(701:800, 151:250) = flipud(r_room);
    
    world_map(701:705, 1:100) = 0;
    world_map(701:705, 151:250) = 0;
end

