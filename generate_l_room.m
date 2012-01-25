function l_room = generate_l_room()
    l_room = zeros(10, 10, 'uint8');
    l_room(:,:) = 255; % set all to white
    l_room(1, :) = 0;
    l_room(1:7,10) = 0;
    l_room(:, 1) = 0;
    l_room(4:7,5:6) = 0;
end