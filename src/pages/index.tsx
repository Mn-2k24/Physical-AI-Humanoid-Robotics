import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs">
            Start Learning 🤖 →
          </Link>
          <Link
            className="button button--outline button--secondary button--lg"
            to="/docs/preface">
            Read Preface
          </Link>
        </div>
        <div className={styles.heroSubtext}>
          <p>
            <strong>No expensive hardware required.</strong> Learn using <strong>ROS 2</strong>, <strong>Gazebo</strong>,
            <strong> Unity</strong>, and <strong>NVIDIA Isaac Sim</strong> on your own computer.
          </p>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Learn to build, simulate, and deploy humanoid robots using ROS 2, NVIDIA Isaac Sim, and Vision-Language-Action models. Hands-on tutorials with production-ready code.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
