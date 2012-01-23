function l_room = generate_l_room()
    l_room = zeros(100, 100, 'uint8');
    l_room(:,:) = 255; % set all to white
    l_room(1:5, :) = 0;
    l_room(1:70,96:100) = 0;
    l_room(:, 1:5) = 0;
    l_room(31:70,41:60) = 0;
end