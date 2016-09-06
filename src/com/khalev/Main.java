package com.khalev;

import cz.cuni.mff.d3s.deeco.annotations.Process;
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
import java.io.OutputStream;
import java.io.PrintStream;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;


public class Main {

    public static void main(String[] args) throws AnnotationProcessorException, SimulationParametersException {
        if (args.length < 1) {
            System.out.println("Please provide a name of XML file with simulation properties");
        } else if (!validateAgainstXSD(args[0], "EfDSchema.xsd")) {
            throw  new SimulationParametersException("Specified file does not represent a valid simulation properties file");
        } else {
            suppressErrors();
            startRuntime(initializeFromXML(args[0]));
        }

    }

    static boolean validateAgainstXSD(String xml, String xsd) {
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

    private static void startRuntime(List<Object> list) throws AnnotationProcessorException {
        AnnotationProcessor processor = new AnnotationProcessor(RuntimeMetadataFactoryExt.eINSTANCE);
        RuntimeMetadata model = RuntimeMetadataFactoryExt.eINSTANCE.createRuntimeMetadata();
        processor.process(model, list);

        RuntimeFrameworkBuilder builder = new RuntimeFrameworkBuilder(
                new RuntimeConfiguration(
                        RuntimeConfiguration.Scheduling.WALL_TIME,
                        RuntimeConfiguration.Distribution.LOCAL,
                        RuntimeConfiguration.Execution.SINGLE_THREADED));

        RuntimeFramework runtime = builder.build(model);
        Environment.getInstance().setRuntime(runtime);
        runtime.start();
    }

    private static List<Object> initializeFromXML(String XMLFile) throws SimulationParametersException {
        try {
            DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
            DocumentBuilder db = dbf.newDocumentBuilder();
            Document doc = db.parse(new File(XMLFile));

            NodeList components = doc.getElementsByTagName("robot");
            ArrayList<RobotPlacement> robots = new ArrayList<>();
            for (int i = 0; i < components.getLength(); i++) {
                Element e = (Element) components.item(i);
                String classname = e.getAttributes().getNamedItem("class").getTextContent();
                double posX = Double.parseDouble(e.getAttributes().getNamedItem("posX").getTextContent());
                double posY = Double.parseDouble(e.getAttributes().getNamedItem("posY").getTextContent());
                double angle = Math.toRadians(Double.parseDouble(e.getAttributes().getNamedItem("angle").getTextContent()));
                DEECoRobot r = createNewRobot(classname);
                robots.add(new RobotPlacement(r, posX, posY, angle));
            }

            List<Object> list = new ArrayList<>();

            for (RobotPlacement rp : robots) {
                list.add(rp.robot);
            }
            list.add(new Coordinator(robots.size()));
            list.add(ActionEnsemble.class);
            list.add(InputEnsemble.class);

            NamedNodeMap nnm = doc.getDocumentElement().getAttributes();
            int sizeX = Integer.parseInt(nnm.getNamedItem("sizeX").getTextContent());
            int sizeY = Integer.parseInt(nnm.getNamedItem("sizeY").getTextContent());
            int cycles = Integer.parseInt(nnm.getNamedItem("cycles").getTextContent());
            File logfile = new File(nnm.getNamedItem("logfile").getTextContent());

            checkParametersForConsistency(robots, sizeX, sizeY, cycles);

            Environment env = new Environment(sizeX, sizeY, cycles, robots, logfile);
            Environment.setInstance(env);

            return list;

        } catch (SAXException | ParserConfigurationException | IOException e) {
            e.printStackTrace();
        }
        return null;
    }

    private static DEECoRobot createNewRobot(String classname) throws SimulationParametersException {

        try {
            Class clazz = Class.forName(classname);
            if (!DEECoRobot.class.isAssignableFrom(clazz)) {
                throw new SimulationParametersException("Class " + classname + " does not extend class DEECoRobot.");
            }
            if (!hasAtLeastOneProcess(clazz.getMethods())) {
                throw new SimulationParametersException("Class " + classname + " does not contain any process.");
            }
            return (DEECoRobot) clazz.newInstance();
        } catch (ClassNotFoundException e) {
            throw new SimulationParametersException("Class " + classname + " does not exist.");
        } catch (InstantiationException | IllegalAccessException e) {
            throw new SimulationParametersException("Class " + classname + " is not valid class.");
        }
    }

    private static boolean hasAtLeastOneProcess(Method[] methods) {
        for (Method m : methods) {
            if (m.isAnnotationPresent(Process.class)) {
                return true;
            }
        }
        return false;
    }

    private static void checkParametersForConsistency(ArrayList<RobotPlacement> robots, int sizeX, int sizeY, int cycles) throws SimulationParametersException {

        if (sizeX > 1000 || sizeY > 1000) {
            throw new SimulationParametersException("The size of simulation can not be bigger than 1000x1000");
        }
        if (cycles > 20000) {
            throw new SimulationParametersException("The number of cycles can not be bigger than 10 000");
        }
        if (robots.size() > 30) {
            throw new SimulationParametersException("You can not create a simulation with more than a 30 robots");
        }

        for (int i = 0; i < robots.size(); i++) {
            RobotPlacement r = robots.get(i);
            if (r.x < Environment.ROBOT_RADIUS || r.x > sizeX - Environment.ROBOT_RADIUS ||
                    r.y < Environment.ROBOT_RADIUS || r.y > sizeY - Environment.ROBOT_RADIUS) {
                throw new SimulationParametersException("There was found a collision between the robot #" + (i+1) +
                        " and the wall. To be consistent, initial parameters of simulation should not contain collisions.");
            }
        }

        for (int i = 0; i < robots.size(); i++) {
            for (int j = 0; j < robots.size(); j++) {
                if (i != j && Environment.distance(robots.get(i), robots.get(j)) < Environment.DOUBLE_RADIUS_SQUARED)
                    throw new SimulationParametersException("There was found a collision between robot #" + (i+1) +
                            " and robot #" + (j+1) + ". To be consistent, initial parameters of simulation should not contain collisions.");
            }
        }
    }

    private static void suppressErrors() {
        System.setErr(new PrintStream(new OutputStream() {
            public void write(int b) {
            }
        }));
    }

}
