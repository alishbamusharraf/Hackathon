import React, { useEffect } from 'react';
import Layout from '@theme/Layout';

export default function Home() {

  // Optional: Floating animation using CSS keyframes
  useEffect(() => {
    const circles = document.querySelectorAll('.floating-circle');
    circles.forEach((circle, index) => {
      const delay = index * 1.5;
      circle.style.animationDelay = `${delay}s`;
    });
  }, []);

  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Learn ROS 2, NVIDIA Isaac, Digital Twins & Vision-Language-Action systems"
    >
      <main
        style={{
          position: 'relative',
          display: 'flex',
          flexDirection: 'column',
          justifyContent: 'center',
          alignItems: 'center',
          padding: '120px 20px',
          textAlign: 'center',
          background: 'linear-gradient(135deg, #e0f2ff 0%, #ffffff 100%)',
          minHeight: '90vh',
          overflow: 'hidden',
          fontFamily: "'Inter', sans-serif",
        }}
      >
        {/* Floating Circles */}
        <div
          style={{
            position: 'absolute',
            top: '-50px',
            left: '-50px',
            width: '300px',
            height: '300px',
            borderRadius: '50%',
            background: 'rgba(79, 70, 229, 0.2)',
            animation: 'float 10s ease-in-out infinite',
          }}
          className="floating-circle"
        ></div>
        <div
          style={{
            position: 'absolute',
            bottom: '-80px',
            right: '-80px',
            width: '400px',
            height: '400px',
            borderRadius: '50%',
            background: 'rgba(99, 102, 241, 0.15)',
            animation: 'float 15s ease-in-out infinite',
          }}
          className="floating-circle"
        ></div>
        <div
          style={{
            position: 'absolute',
            top: '30%',
            right: '10%',
            width: '200px',
            height: '200px',
            borderRadius: '50%',
            background: 'rgba(79, 70, 229, 0.1)',
            animation: 'float 12s ease-in-out infinite',
          }}
          className="floating-circle"
        ></div>

        {/* Hero Section */}
        <h1
          style={{
            fontSize: '3rem',
            fontWeight: '900',
            marginBottom: '25px',
            color: '#111827',
            maxWidth: '900px',
            lineHeight: '1.2',
            animation: 'fadeInUp 1s ease forwards',
            opacity: 0,
          }}
        >
          <span style={{ background: 'linear-gradient(90deg, #6366f1, #4f46e5)', WebkitBackgroundClip: 'text', color: 'transparent' }}>
            Physical AI & Humanoid Robotics
          </span>
        </h1>

        <p
          style={{
            fontSize: '1.25rem',
            maxWidth: '750px',
            color: '#4b5563',
            marginBottom: '50px',
            lineHeight: '1.7',
            animation: 'fadeInUp 1.5s ease forwards',
            opacity: 0,
          }}
        >
          Step-by-step labs and examples covering ROS 2, digital twin simulation, NVIDIA Isaac, motor control, and Vision-Language-Action systems.
        </p>

        {/* Buttons */}
        <div style={{ display: 'flex', gap: '20px', flexWrap: 'wrap', justifyContent: 'center', animation: 'fadeInUp 2s ease forwards', opacity: 0 }}>
          <a
            href="/docs/intro"
            style={{
              padding: '16px 32px',
              backgroundColor: '#4f46e5',
              color: 'white',
              borderRadius: '12px',
              textDecoration: 'none',
              fontSize: '1.1rem',
              fontWeight: '600',
              boxShadow: '0 8px 24px rgba(79, 70, 229, 0.25)',
              transition: 'all 0.3s ease',
            }}
            onMouseOver={(e) => {
              e.target.style.transform = 'scale(1.07)';
              e.target.style.boxShadow = '0 12px 28px rgba(79, 70, 229, 0.35)';
            }}
            onMouseOut={(e) => {
              e.target.style.transform = 'scale(1)';
              e.target.style.boxShadow = '0 8px 24px rgba(79, 70, 229, 0.25)';
            }}
          >
            🚀 Start Learning
          </a>

          <a
            href="https://github.com/LAIBAMUSHARAF45/Hackathon-book"
            target="_blank"
            style={{
              padding: '16px 32px',
              border: '2px solid #4f46e5',
              borderRadius: '12px',
              color: '#4f46e5',
              textDecoration: 'none',
              fontSize: '1.1rem',
              fontWeight: '600',
              transition: 'all 0.3s ease',
            }}
            onMouseOver={(e) => {
              e.target.style.backgroundColor = '#eef2ff';
              e.target.style.transform = 'scale(1.07)';
              e.target.style.borderColor = '#6366f1';
            }}
            onMouseOut={(e) => {
              e.target.style.backgroundColor = 'transparent';
              e.target.style.transform = 'scale(1)';
              e.target.style.borderColor = '#4f46e5';
            }}
          >
            ⭐ View on GitHub
          </a>
        </div>

        {/* Footer */}
        <p
          style={{
            marginTop: '70px',
            color: '#6b7280',
            fontSize: '1rem',
            maxWidth: '700px',
            lineHeight: '1.6',
            animation: 'fadeInUp 2.5s ease forwards',
            opacity: 0,
          }}
        >
          Use the sidebar to explore each module and build your robotics journey.
        </p>

        {/* Keyframes */}
        <style>{`
          @keyframes float {
            0% { transform: translateY(0px) translateX(0px); }
            50% { transform: translateY(-20px) translateX(15px); }
            100% { transform: translateY(0px) translateX(0px); }
          }

          @keyframes fadeInUp {
            0% { opacity: 0; transform: translateY(20px); }
            100% { opacity: 1; transform: translateY(0); }
          }
        `}</style>
      </main>
    </Layout>
  );
}
