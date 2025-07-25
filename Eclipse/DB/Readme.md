create database turtledb;
create user turtle@localhost identified by 'turtle';
grant all privileges on turtledb.* to turtle@localhost;
commit;
