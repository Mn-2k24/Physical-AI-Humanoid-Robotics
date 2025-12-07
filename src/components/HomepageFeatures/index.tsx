import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  emoji: string;
  description: ReactNode;
  link: string;
};

const ModuleList: FeatureItem[] = [
  {
    title: 'Module 1: ROS 2 Middleware',
    emoji: '🔧',
    link: '/docs/module-1-ros2',
    description: (
      <>
        Master the communication backbone for robotics. Learn ROS 2 nodes, topics, services,
        and URDF robot modeling. Build your first "Hello World" system with Python rclpy.
      </>
    ),
  },
  {
    title: 'Module 2: Digital Twin Simulation',
    emoji: '🌐',
    link: '/docs/module-2-simulation',
    description: (
      <>
        Test robot behaviors in realistic physics environments. Use Gazebo and Unity
        for simulation, sensor modeling (LiDAR, cameras, IMU), and safe prototyping.
      </>
    ),
  },
  {
    title: 'Module 3: NVIDIA Isaac Platform',
    emoji: '🚀',
    link: '/docs/module-3-isaac',
    description: (
      <>
        Leverage GPU-accelerated AI for perception and navigation. Learn Isaac Sim,
        synthetic data generation, VSLAM, object detection, and Nav2 path planning.
      </>
    ),
  },
  {
    title: 'Module 4: Vision-Language-Action',
    emoji: '🗣️',
    link: '/docs/module-4-vla',
    description: (
      <>
        Enable natural language control of humanoid robots. Integrate speech recognition
        (Whisper), cognitive planning with LLMs, and build a voice-controlled capstone project.
      </>
    ),
  },
];

function Feature({title, emoji, description, link}: FeatureItem) {
  return (
    <div className={clsx('col col--6')}>
      <Link to={link} className={styles.featureLink}>
        <div className={styles.featureCard}>
          <div className="text--center">
            <div className={styles.featureEmoji}>{emoji}</div>
          </div>
          <div className="text--center padding-horiz--md">
            <Heading as="h3">{title}</Heading>
            <p>{description}</p>
          </div>
        </div>
      </Link>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={clsx('text--center', styles.featuresHeader)}>
          <Heading as="h2">Four Progressive Modules</Heading>
          <p className="hero__subtitle">
            From ROS 2 basics to advanced VLA-controlled humanoid systems
          </p>
        </div>
        <div className="row">
          {ModuleList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
        <div className={clsx('text--center', styles.featuresFooter)}>
          <Link
            className="button button--primary button--lg"
            to="/docs/learning-objectives">
            View Learning Objectives →
          </Link>
        </div>
      </div>
    </section>
  );
}
