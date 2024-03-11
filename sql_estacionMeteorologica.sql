create database EstacionMeteorologico;
use EstacionMeteorologico;
select * from weather_measurements; 

drop table ranning;
drop table weather_measurements;

create table weather_measurements(

	id_weather int auto_increment primary key,
	temperature decimal(10,2),
	pressure decimal(10,2),
	altitude decimal(10,2),
	pm1_0 decimal(10,2),
	pm2_5 decimal(10,2),
	pm10 decimal(10,2),
	ozone decimal(10,2),
	air_quality decimal(10,2),
	uv decimal(10,2),
	windSpeed decimal(10,2),
	windDirection varchar(15), 
	date_time datetime

);

drop procedure sp_StoreMeteorologicalData ;
--  procedimiento almacenado para guardar información en la base de datos 
delimiter //
create procedure sp_StoreMeteorologicalData(

	in Temperature float , 
    in Pressure float, 
    in Altitude float, 
    in Pm1_0 float , 
    in Pm2_5 float ,
    in Pm10 float,
    in Ozone  float ,
    in Air_quality float,
    in Uv float,
    in WindSpeed float,
    in WindDirection varchar(15)
    )
	begin
    
		Insert into weather_measurements (temperature, pressure, altitude,pm1_0,pm2_5,pm10, ozone,  air_quality, uv, windSpeed, windDirection, date_time) 
        values (Temperature,Pressure , Altitude , Pm1_0 , Pm2_5  , Pm10 , Ozone, Air_quality,  Uv , WindSpeed , WindDirection , now());
        
	end //
delimiter ;

create table rain(
id int auto_increment primary key ,
lm2 float ,
tds float , 
date_time datetime
);

CALL sp_StoreMeteorologicalData(
    25.5,   -- Temperature
    1013.2, -- Pressure
    500,    -- Altitude
    5.6,    -- Pm1_0
    10.3,   -- Pm2_5
    15.8,   -- Pm10
    0.035,  -- Ozone
    80,     -- Air_quality
    7.2,    -- Uv
    15.5,   -- WindSpeed
    'N'     -- WindDirection
);

CALL sp_StoreMeteorologicalData(
    28.8,   -- Temperature
    1012.5, -- Pressure
    600,    -- Altitude
    8.2,    -- Pm1_0
    12.7,   -- Pm2_5
    18.4,   -- Pm10
    0.042,  -- Ozone
    75,     -- Air_quality
    8.9,    -- Uv
    18.2,   -- WindSpeed
    'SW'    -- WindDirection
);
select * from weather_measurements;

delimiter //
create procedure sp_StoreRanningData( in liters decimal, in  tdsContamiant decimal )
begin 
	Insert into rain( lm2,tds,date_time) values (liters, tdsContamiant, now() );
end //
delimiter ;

/* me permite ver la cantidad de registros por 6 horas  */
select * from rain ;
create view sp_view_dataRain as
      select * from rain 
      where hour(date_time) between  (hour(curtime())-5) and hour(curtime()) 
      group by HOUR(date_time)  order by  date_time DESC limit 6;
      
      
select * from sp_view_dataRain;      
select * from weather_measurements; 
drop view sp_view_weatherdata;
 create view sp_view_weatherdata as
      select *, DATE_FORMAT(date_time, '%H:%i:%S') AS hora  from weather_measurements 
      where hour(date_time) between  (hour(curtime())-5) and hour(curtime()) 
      group by HOUR(date_time)  order by  date_time DESC limit 6;     
	
    
    
select curtime()AS hora_Actual , hour(curtime()) ;
SELECT DATE_FORMAT(NOW(), '%H') , now();



call sp_StoreMeteorologicalData(40,40,40,40);
select * from estacionmeteorologico.weather_measurements;

create view get_RealTimeData as
select * from weather_measurements order by  date_time desc limit 1 ;

-- este metodo te dice todos los datos por mes
create view get_DataByMonth as 
select * from weather_measurements where month(date_time) = month(now());

 -- este procedimiento borrar los datos del mes pasado
 delimiter //
 
 create procedure ps_deleteHistory()
 begin
	 if (select day(now())) = 1 then
		delete from weather_measurements where month(date_time) = month(now())-1;
	end if;
 end //
 delimiter ;
drop table rain;

drop procedure sp_storeLitrosm2 ;
delimiter //
create procedure sp_storeLitrosm2( in  LM2 float , in tds float  )
begin 
	insert into pluviometro (lm2,tds,date_time) values (LM2,TDS, now());
end //
delimiter ;


drop procedure sp_storeLitrosm2;
call sp_storeLitrosm2(23.4);
select * from pluviometro;
select * from weather_measurements;






/*
	INSERT INTO `estacionmeteorologico`.`rain` (`id`, `lm2`, `tds`, `date_time`) VALUES ('', '3.2', '156', '2024-02-29 06:30:00');
INSERT INTO `estacionmeteorologico`.`rain` (`id`, `lm2`, `tds`, `date_time`) VALUES ('', '13.2', '152', '2024-02-29 06:50:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('515', '153', '2024-02-29 07:30:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('1515', '154', '2024-02-29 07:50:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('26', '152', '2024-02-29 07:55:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('5256.2', '15', '2024-02-29 08:30:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('15', '1', '2024-02-29 08:50:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('48', '2', '2024-02-29 09:30:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('78', '3', '2024-02-29 09:50:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('89', '1', '2024-02-29 10:30:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('56', '4', '2024-02-29 10:40:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('15', '8', '2024-02-29 10:50:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('15', '56', '2024-02-29 11:30:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('15', '45', '2024-02-29 11:30:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('894', '65', '2024-02-29 11:30:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('489', '158', '2024-02-29 12:30:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('48', '51', '2024-02-29 12:30:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('48', '561', '2024-02-29 12:30:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('498', '15', '2024-02-29 13:20:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('15', '189', '2024-02-29 13:30:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('148', '8', '2024-02-29 13:50:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('485', '84', '2024-02-29 14:20:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('51', '454', '2024-02-29 14:30:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('8', '4', '2024-02-29 14:50:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('96', '15', '2024-02-29 15:10:00');
INSERT INTO `estacionmeteorologico`.`rain` (`lm2`, `tds`, `date_time`) VALUES ('548', '515', '2024-02-29 15:20:00');


*/


