package com.khalev.efd.simulation;

import cz.cuni.mff.d3s.deeco.annotations.Component;
import cz.cuni.mff.d3s.deeco.annotations.Ensemble;
import cz.cuni.mff.d3s.deeco.annotations.processor.AnnotationProcessor;
import cz.cuni.mff.d3s.deeco.annotations.processor.AnnotationProcessorException;
import cz.cuni.mff.d3s.deeco.model.runtime.api.RuntimeMetadata;
import cz.cuni.mff.d3s.deeco.model.runtime.custom.RuntimeMetadataFactoryExt;
import cz.cuni.mff.d3s.deeco.runtime.RuntimeConfiguration;
import cz.cuni.mff.d3s.deeco.runtime.RuntimeFramework;
import cz.cuni.mff.d3s.deeco.runtime.RuntimeFrameworkBuilder;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NamedNodeMap;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import javax.xml.XMLConstants;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.stream.StreamSource;
import javax.xml.validation.Schema;
import javax.xml.validation.SchemaFactory;
import javax.xml.validation.Validator;
import java.io.File;
import java.io.IOException;
import java.lang.annotation.Annotation;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;

public class Simulation {

    private String schema = "EfDSchema.xsd";
    private List<Object> components = new ArrayList<>();
    private ArrayList<RobotPlacement> robots = new ArrayList<>();
    private ArrayList<SensoryInputsProcessor> sensors = new ArrayList<>();

    private File logfile;
    private File bitmap;
    private EnvironmentMap map;
    private int cycles;

    private static int CYCLE = 2;

    static int getCYCLE() {
        return CYCLE;
    }

    public Simulation(String parametersFile) throws SimulationParametersException {
        if (!validateAgainstXSD(parametersFile, schema)) {
            throw  new SimulationParametersException("Specified file does not represent a valid simulation properties file");
        }
        initializeFromXML(parametersFile);
    }

    public void startSimulation() throws AnnotationProcessorException {
        Environment env = new Environment(cycles, robots, logfile, map, bitmap.getAbsolutePath(), sensors);
        Environment.setInstance(env);
        startRuntime();
    }

    private boolean validateAgainstXSD(String xml, String xsd) {
        try {
            SchemaFactory factory = SchemaFactory.newInstance(XMLConstants.W3C_XML_SCHEMA_NS_URI);
            Schema schema = factory.newSchema(new StreamSource(new File(xsd)));
            Validator validator = schema.newValidator();
            validator.validate(new StreamSource(new File(xml)));
            return true;
        } catch(Exception ex) {
            return false;
        }
    }



    private void startRuntime() throws AnnotationProcessorException {
        AnnotationProcessor processor = new AnnotationProcessor(RuntimeMetadataFactoryExt.eINSTANCE);
        RuntimeMetadata model = RuntimeMetadataFactoryExt.eINSTANCE.createRuntimeMetadata();
        processor.process(model, components);

        RuntimeFrameworkBuilder builder = new RuntimeFrameworkBuilder(new RuntimeConfiguration(
                RuntimeConfiguration.Scheduling.WALL_TIME, RuntimeConfiguration.Distribution.LOCAL,
                RuntimeConfiguration.Execution.SINGLE_THREADED));

        RuntimeFramework runtime = builder.build(model);
        Environment.getInstance().setRuntime(runtime);
        runtime.start();
    }

    private void initializeFromXML(String XMLFile) throws SimulationParametersException {
        try {
            DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
            DocumentBuilder db = dbf.newDocumentBuilder();
            Document doc = db.parse(new File(XMLFile));

            //Processing attributes
            NamedNodeMap nnm = doc.getDocumentElement().getAttributes();
            cycles = Integer.parseInt(nnm.getNamedItem("cycles").getTextContent());
            if (nnm.getNamedItem("processingTime") != null) {
                CYCLE = Integer.parseInt(nnm.getNamedItem("processingTime").getTextContent());
            }
            logfile = new File(nnm.getNamedItem("logfile").getTextContent());
            bitmap = new File(nnm.getNamedItem("bitmap").getTextContent());

            BitmapProcessor bp = new BitmapProcessor(bitmap);
            map = bp.readBitmap();

            //Processing robots
            NodeList robotList = doc.getElementsByTagName("robot");
            for (int i = 0; i < robotList.getLength(); i++) {
                Element e = (Element) robotList.item(i);
                String classname = e.getAttributes().getNamedItem("class").getTextContent();
                double posX = Double.parseDouble(e.getAttributes().getNamedItem("posX").getTextContent());
                double posY = Double.parseDouble(e.getAttributes().getNamedItem("posY").getTextContent());
                double angle = Math.toRadians(Double.parseDouble(e.getAttributes().getNamedItem("angle").getTextContent()));
                DEECoRobot r = createNewRobot(classname);
                components.add(r);
                robots.add(new RobotPlacement(r, posX, posY, angle));
            }
            //Processing ensembles
            NodeList ensembles = doc.getElementsByTagName("ensemble");
            for (int i = 0; i < ensembles.getLength(); i++) {
                Element e = (Element) ensembles.item(i);
                String classname = e.getAttributes().getNamedItem("class").getTextContent();
                Class ensemble = loadEnsemble(classname);
                components.add(ensemble);
            }
            //Processing sensors
            ArrayList<String> sensorNames = new ArrayList<>();
            sensorNames.add("collisions");

            NodeList sensors = doc.getElementsByTagName("sensor");
            for (int i = 0; i < sensors.getLength(); i++) {
                Element e = (Element) sensors.item(i);
                String classname = e.getAttributes().getNamedItem("processor").getTextContent();
                SensoryInputsProcessor sip = createDataProcessor(classname);
                this.sensors.add(sip);
                sensorNames.add(e.getAttributes().getNamedItem("name").getTextContent());
            }
            components.add(new Coordinator(robots.size(), sensorNames));
            components.add(ActionEnsemble.class);
            components.add(InputEnsemble.class);

            checkParametersForConsistency(robots, map, cycles);
        } catch (SAXException | ParserConfigurationException | IOException e) {
            throw new RuntimeException(e);
        }
    }

    private DEECoRobot createNewRobot(String classname) throws SimulationParametersException {
        try {
            Class clazz = Class.forName(classname);
            Annotation annotation = clazz.getAnnotation(Component.class);
            if (annotation == null) {
                throw new SimulationParametersException("Class " + classname + " is not a DEECo @Component");
            }
            if (!DEECoRobot.class.isAssignableFrom(clazz)) {
                throw new SimulationParametersException("Class " + classname + " does not extend class DEECoRobot.");
            }
            return (DEECoRobot) clazz.newInstance();
        } catch (ClassNotFoundException e) {
            throw new SimulationParametersException("Class " + classname + " does not exist.");
        } catch (InstantiationException | IllegalAccessException e) {
            throw new SimulationParametersException("Class " + classname + " is not valid class.");
        }
    }

    //TODO: exception handling
    private SensoryInputsProcessor createDataProcessor(String classname) throws SimulationParametersException {
        try {
            Class cls = Class.forName(classname);
            Constructor cons = cls.getConstructor(EnvironmentMap.class);
            if (!SensoryInputsProcessor.class.isAssignableFrom(cls)) {
                throw new SimulationParametersException("Class " + classname + " does not extend class SensoryInputsProcessor.");
            }
            return (SensoryInputsProcessor) cons.newInstance(map);
        } catch (ClassNotFoundException e) {
            throw new SimulationParametersException("Class " + classname + " does not exist.");
        } catch (InstantiationException | IllegalAccessException | NoSuchMethodException | InvocationTargetException e) {
            throw new SimulationParametersException("Class " + classname + " is not valid class.");
        }
    }

    private Class loadEnsemble(String classname) throws SimulationParametersException {
        try {
            Class cls = Class.forName(classname);
            Annotation annotation = cls.getAnnotation(Ensemble.class);
            if (annotation == null) {
                throw new SimulationParametersException("Class " + classname + " is not a DEECo @Ensemble");
            }
            return cls;
        } catch (ClassNotFoundException e) {
            throw new SimulationParametersException("Class " + classname + " does not exist.");
        }
    }

    private void checkParametersForConsistency(ArrayList<RobotPlacement> robots, EnvironmentMap map, int cycles) throws SimulationParametersException {

        if (map.sizeX > 1000 || map.sizeY > 1000) {
            throw new SimulationParametersException("The size of simulation cannot be bigger than 1000x1000");
        }
        if (cycles > 20000) {
            throw new SimulationParametersException("The number of cycles can not be bigger than 10 000");
        }
        if (robots.size() > 30) {
            throw new SimulationParametersException("You can not create a simulation with more than a 30 robots");
        }

        Collision collision = SimulationEngine.checkMapConsistency(robots, map);
        if (collision.type == Collision.Type.WALL) {
            throw new SimulationParametersException("There was found a collision between the robot #" + (collision.num1+1) +
                    " and the wall. To be consistent, initial parameters of simulation should not contain collisions.");
        } else if (collision.type == Collision.Type.ROBOT) {
            throw new SimulationParametersException("There was found a collision between robot #" + (collision.num1+1) +
                    " and robot #" + (collision.num2+1) + ". To be consistent, initial parameters of simulation should not contain collisions.");
        }

    }
}
